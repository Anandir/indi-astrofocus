/*******************************************************************************
  Copyright(c) 2020 Giacomo Succ. All rights reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.


 * AstroFocus 5 serial commands list
 * -------------------------------
 * 0,0  Returns current position
 * 0,N  Sets the current position
 * 1,N	Go to absolute position N
 * 2,N	Go to relative position N ignoring limits
 * 3,0	Sets the current position as the lower limit (point 0)
 * 4,0	Returns the current upper limit
 * 4,1	Sets the current position as te upper limit
 * 4,N	Sets the upper limit to N (> 1) (*)
 * 5,0	Returns T if the temperature sensor is present, otherwise F
 * 5,1	Returns the current temperature
 * 6,0	Returns the temperature coefficient
 * 6,N	Sets the temperature coefficient to N
 * 7,0	Disables temperature compensation
 * 7,1	Activates temperature compensation
 * 8,0  Returns the step size in 1/100 micron
 * 8,N	Sets the step size in 1/100 micron
 * 9,0	Returns the version of the program
 * 10,0 Returns stepper motor power (1-255)
 * 10,N Sets stepper motor power (1-255)
 * 11,0 Returns pulses duration in milliseconds
 * 11,N Sets duration of the pulses in milliseconds
 * 12,0 Returns the pause before the stepper motor power cut in milliseconds
 * 12,N Sets the pause before the stepper motor power cut in milliseconds
 * 13,0 Returns the motion mode (1: One Step Full Step, 2: Two Phase Full Step, 3: Half Step)
 * 13,N Sets the motion mode (1: One Step Full Step, 2: Two Phase Full Step, 3: Half Step)

 * Additional notes
 * -------------------------------
 * Every command need to have an '\n' at the end before send.
 * Every message returned by the focuser is terminated by a '\n'.

 ***********************************************************************************/

#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <indicom.h>
#include "connectionplugins/connectionserial.h"

#include "astrofocus_focuser.h"

static std::unique_ptr<AstrofocusFocuser> astrofocusFocuser(new AstrofocusFocuser());

/**************************************************************************************
 ** Constructor
 ***************************************************************************************/
AstrofocusFocuser::AstrofocusFocuser()
{
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE);
    
    // -------

    setSupportedConnections(CONNECTION_SERIAL);
    
    // -------

    setVersion(AFF_MAJOR_VERSION, AFF_MINOR_VERSION);
}

/**************************************************************************************
 ** Distructor
 ***************************************************************************************/
AstrofocusFocuser::~AstrofocusFocuser()
{
}

/**************************************************************************************
 ** INDI is asking us for our default device name
 ***************************************************************************************/
const char * AstrofocusFocuser::getDefaultName()
{
    return "Astrofocus";
}

/**************************************************************************************
 ** 
 ***************************************************************************************/
bool AstrofocusFocuser::initProperties()
{
    INDI::Focuser::initProperties();

    #ifdef DEBUG_BUILD
    addDebugControl();
    #endif

    // -------

    IUFillText(&FirmwareVersionT[0], "FIRMWARE_VERSION_TEXT", "Firmware Version", "");
    IUFillTextVector(&FirmwareVersionTP, FirmwareVersionT, 1, getDeviceName(), "FIRMWARE_VERSION",
                     "Firmware Version", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // -------

    IUFillNumber(&StepSizeN[0], "STEP_SIZE_TEXT", "Step Size [1/100 micron]", "%d", -32768., 32767., 1., 0.);
    IUFillNumberVector(&StepSizeNP, StepSizeN, 1, getDeviceName(), "STEP_SIZE", "Step Size", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    // -------

    IUFillSwitch(&StepperModeS[STEPPER_MODE_ONE_PHASE_FULL_STEP], "One Phase Full Step", "One Phase Full Step", ISS_ON);
    IUFillSwitch(&StepperModeS[STEPPER_MODE_TWO_PHASE_FULL_STEP], "Two Phase Full Step", "Two Phase Full Step", ISS_OFF);
    IUFillSwitch(&StepperModeS[STEPPER_MODE_HALF_STEP], "Half Step", "Half Step", ISS_OFF);
    IUFillSwitchVector(&StepperModeSP, StepperModeS, STEPPER_MODE_COUNT, getDeviceName(),
                       "STEPPER_MODE", "Stepper Mode", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY,
                       60, IPS_IDLE);
    
    return true;
}

/**************************************************************************************
 ** 
 ***************************************************************************************/
bool AstrofocusFocuser::updateProperties()
{
    INDI::Focuser::updateProperties();
    
    if (isConnected())
    {
        loadSettingsFromDevice();
        
        defineNumber(&StepSizeNP);
        defineText(&FirmwareVersionTP);
        defineSwitch(&StepperModeSP);
    }
    else
    {
        deleteProperty(StepSizeNP.name);
        deleteProperty(FirmwareVersionTP.name);
        deleteProperty(StepperModeSP.name);
    }
    
    return true;
}

/**************************************************************************************
 ** Return properties of device.
 ***************************************************************************************/
void ISGetProperties (const char *dev)
{
    astrofocusFocuser->ISGetProperties(dev);
}

void AstrofocusFocuser::ISGetProperties(const char *dev)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) != 0)
        return;

    INDI::Focuser::ISGetProperties(dev);

    //defineSwitch(&ModeSP);
    //loadConfig(true, "Mode");
}

/**************************************************************************************
 ** Process new switch from client
 ***************************************************************************************/
void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
    astrofocusFocuser->ISNewSwitch(dev, name, states, names, num);
}

bool AstrofocusFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (!strcmp(name, StepperModeSP.name))
        {
            int currentIndex = IUFindOnSwitchIndex(&StepperModeSP);

            switch (currentIndex)
            {
                case STEPPER_MODE_ONE_PHASE_FULL_STEP:
                {
                    sendCommand("13,1");
                    break;
                }
                case STEPPER_MODE_TWO_PHASE_FULL_STEP:
                {
                    sendCommand("13,2");
                    break;
                }
                case STEPPER_MODE_HALF_STEP:
                {
                    sendCommand("13,3");
                    break;
                }
                default:
                {
                    StepperModeSP.s = IPS_ALERT;
                    IDSetSwitch(&StepperModeSP, "AstrofocusFocuser::ISNewSwitch => Unknown mode index %d", currentIndex);
                    return true;
                }
            }
 
            if(!receivedAck())
            {
                StepperModeSP.s = IPS_ALERT;
                IDSetSwitch(&StepperModeSP, "AstrofocusFocuser::ISNewSwitch => Ack not received for index %d", currentIndex);
                return false;
            }

            IUUpdateSwitch(&StepperModeSP, states, names, n);
            currentIndex = IUFindOnSwitchIndex(&StepperModeSP);
            StepperModeSP.s = IPS_OK;
            IDSetSwitch(&StepperModeSP, NULL);

            DEBUGF(INDI::Logger::DBG_SESSION, "AstrofocusFocuser::ISNewSwitch => The new value is %s", StepperModeS[currentIndex].label);
            
            return true;
        }
    }

    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}
 
/**************************************************************************************
 ** Process new text from client
 ***************************************************************************************/
void ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
    astrofocusFocuser->ISNewText(dev, name, texts, names, n);
}

/**************************************************************************************
 ** Process new number from client
 ***************************************************************************************/
void ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
    astrofocusFocuser->ISNewNumber(dev, name, values, names, n);
}
 
/**************************************************************************************
 ** Process new blob from client
 ***************************************************************************************/
void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}
 
/**************************************************************************************
 ** Process snooped property from another driver
 ***************************************************************************************/
void ISSnoopDevice (XMLEle *root)
{
    astrofocusFocuser->ISSnoopDevice(root);
}

/* ************************************************************************************ */

bool AstrofocusFocuser::Handshake()
{
    int nbytes_read = 0, err_code = 0;
    char response[MESSAGE_MAX_LENGHT];
    char error_message[MAXRBUF];

    tcflush(PortFD, TCIOFLUSH);

    sendCommand("9,0");

    if ((err_code = tty_read_section(PortFD, response, '\n', READ_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(err_code, error_message, MAXRBUF);

        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocuser::Handshake => TTY read error detected: %s", error_message);

        return false;
    }

    if (nbytes_read <= 0)
        return false;

    FirmwareVersionT[0].text = response;
    defineText(&FirmwareVersionTP);

    return true;
}

/**************************************************************************************
 ** Serial communications
 ***************************************************************************************/
int AstrofocusFocuser::sendCommand(const char *cmd)
{
    int nbytes_written = 0, err_code = 0;
    char err_msg[MAXRBUF];
    char cmd_to_send[MESSAGE_MAX_LENGHT];
    
    strcpy(cmd_to_send, cmd);
    strcat(cmd_to_send, "\n");

    tcflush(PortFD, TCIOFLUSH);

    if ((err_code = tty_write_string(PortFD, cmd_to_send, &nbytes_written) != TTY_OK))
    {
        tty_error_msg(err_code, err_msg, MAXRBUF);

        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocuser::sendCommand => cmd_to_send: %s", cmd_to_send);
        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocuser::sendCommand => TTY error detected: %s", err_msg);
        return -1;
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "AstrofocusFocuser::sendCommand => Command sent successfully: %s", cmd_to_send);

    return nbytes_written;
}

/* ************************************************************************************ */

bool AstrofocusFocuser::receivedAck()
{
    char *response;
    bool res = false;
    
    response = receiveResponse();

    if (response == NULL)
    {
        free(response);
        return res;
    }
    
    res = strcmp(response, "OK");
    free(response);

    return res;
}

/* ************************************************************************************ */

char * AstrofocusFocuser::receiveResponse()
{
    int nbytes_read = 0, err_code = 0;
    char error_message[MAXRBUF];
    char *response = (char *)calloc(MESSAGE_MAX_LENGHT, sizeof(char));

    tcflush(PortFD, TCIOFLUSH);

    if ((err_code = tty_read_section(PortFD, response, '\n', READ_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(err_code, error_message, MAXRBUF);

        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocuser::receiveResult => TTY read error detected: %s", error_message);

        return NULL;
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "AstrofocusFocuser::receiveResult => Response: %s", response);

    return response;
}

/* ************************************************************************************ */

void AstrofocusFocuser::loadSettingsFromDevice()
{
    bool has_errors = false, has_temperature_sensor = false;
    char *tmp_buffer;
    int current_position = 0, current_upper_limit = 0, current_temperature_coefficient = 0,
        current_step_size = 0, current_stepper_power = 0, current_pulses_duration = 0,
        current_pause = 0, current_motion_mode = 0;
    float current_temperature = 0;

    // Current position
    sendCommand("0,0");
    
    tmp_buffer = receiveResponse();
    current_position = stringToInt(tmp_buffer, &has_errors);

    if (has_errors)
        current_position = 0;

    free(tmp_buffer);

    // Current upper limit
    sendCommand("4,0");
    
    tmp_buffer = receiveResponse();
    current_upper_limit = stringToInt(tmp_buffer, &has_errors);

    if (has_errors)
        current_upper_limit = 0;

    free(tmp_buffer);
    
    // Current temperature
    sendCommand("5,0");
    tmp_buffer = receiveResponse();

    // T = Sensor is present, 5,1 to gather the temperature
    // F = No sensor, so I can ignore it
    if(strcmp(tmp_buffer, "T"))
        has_temperature_sensor = true;
    else if(strcmp(tmp_buffer, "F"))
        has_temperature_sensor = false;
    else
    {
        // This should never happens
        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocuser::loadSettingsFromDevice => 5,0 unknown response: %s", tmp_buffer);
        has_temperature_sensor = false;
    }

    free(tmp_buffer);

    if (has_temperature_sensor)
    {
        sendCommand("5,1");
        tmp_buffer = receiveResponse();
        
        current_temperature = stringToFloat(tmp_buffer, &has_errors);

        if (has_errors)
            current_temperature = 0;

        free(tmp_buffer);
    }
    
    // Current temmperature coefficient
    sendCommand("6,0");
    tmp_buffer = receiveResponse();
    current_temperature_coefficient = stringToInt(tmp_buffer, &has_errors);

    if (has_errors)
        current_temperature_coefficient = 0;

    free(tmp_buffer);

    // Step size (1/100 micron)
    sendCommand("8,0");
    tmp_buffer = receiveResponse();
    current_step_size = stringToInt(tmp_buffer, &has_errors);

    if (has_errors)
        current_step_size = 0;

    StepSizeN[0].value = (current_step_size / 100);
    /*StepSizeNP.s = IPS_OK;

    if (IUUpdateNumber(&StepSizeNP, values, names, (current_step_size / 100)) == 0)
        IDSetNumber(&StepSizeNP, nullptr);*/

    free(tmp_buffer);
    
    // Stepper motor power (1-255)
    sendCommand("10,0");
    tmp_buffer = receiveResponse();
    current_stepper_power = stringToInt(tmp_buffer, &has_errors);

    if (has_errors)
        current_stepper_power = 0;
    else
    {
        if(current_stepper_power > 255)
        {
            current_stepper_power = 255;
            DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocuser::loadSettingsFromDevice => 10,0 value over the limit: %s", tmp_buffer);
        }
        else if(current_stepper_power < 0)
        {
            current_stepper_power = 0;
            DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocuser::loadSettingsFromDevice => 10,0 value below the limit: %s", tmp_buffer);
        }
    }

    free(tmp_buffer);
    
    // Pulses duration (milliseconds)
    sendCommand("11,0");
    tmp_buffer = receiveResponse();
    current_pulses_duration = stringToInt(tmp_buffer, &has_errors);

    if (has_errors)
        current_pulses_duration = 0;

    free(tmp_buffer);
    
    // Pause before power cutoff (milliseconds)
    sendCommand("12,0");
    tmp_buffer = receiveResponse();
    current_pause = stringToInt(tmp_buffer, &has_errors);

    if (has_errors)
        current_pause = 0;

    free(tmp_buffer);
    
    // Motion mode
    sendCommand("13,0");
    tmp_buffer = receiveResponse();
    current_motion_mode = stringToInt(tmp_buffer, &has_errors);

    if (has_errors)
        current_motion_mode = 1;

    StepperModeS[STEPPER_MODE_ONE_PHASE_FULL_STEP].s = ISS_OFF;
    StepperModeS[STEPPER_MODE_TWO_PHASE_FULL_STEP].s = ISS_OFF;
    StepperModeS[STEPPER_MODE_HALF_STEP].s = ISS_OFF;

    switch(current_motion_mode)
    {
        case 1:
        {
            StepperModeS[STEPPER_MODE_ONE_PHASE_FULL_STEP].s = ISS_ON;
            break;
        }
        case 2:
        {
            StepperModeS[STEPPER_MODE_TWO_PHASE_FULL_STEP].s = ISS_ON;
            break;
        }
        case 3:
        {
            StepperModeS[STEPPER_MODE_HALF_STEP].s = ISS_ON;
            break;
        }
    }
    
    StepperModeSP.s = IPS_OK;
    IDSetSwitch(&StepperModeSP, nullptr);

    free(tmp_buffer);
    
    // -------

    FocusSpeedN[0].min = 0.;
    FocusSpeedN[0].max = 0.;
    FocusSpeedN[0].value = 0.;
    FocusSpeedN[0].step = 0.;
    FocusSpeedNP.s = IPS_OK;

    IDSetNumber(&FocusSpeedNP, nullptr);

    // -------

    FocusTimerN[0].min = 0.;
    FocusTimerN[0].max = 0.;
    FocusTimerN[0].value = 0.;
    FocusTimerN[0].step = 0.;
    FocusTimerNP.s = IPS_OK;

    IDSetNumber(&FocusTimerNP, nullptr);

    // -------

    FocusAbsPosN[0].min = 0.;
    FocusAbsPosN[0].max = 0.;
    FocusAbsPosN[0].value = 0.;
    FocusAbsPosN[0].step = 0.;
    FocusAbsPosNP.s = IPS_OK;

    IDSetNumber(&FocusAbsPosNP, nullptr);

    // -------

    FocusMaxPosN[0].min = 0.;
    FocusMaxPosN[0].max = 0.;
    FocusMaxPosN[0].value = 0.;
    FocusMaxPosN[0].step = 0.;
    FocusMaxPosNP.s = IPS_OK;

    IDSetNumber(&FocusMaxPosNP, nullptr);

    // -------

    FocusSyncN[0].min = 0.;
    FocusSyncN[0].max = 0.;
    FocusSyncN[0].value = 0.;
    FocusSyncN[0].step = 0.;
    FocusSyncNP.s = IPS_OK;

    IDSetNumber(&FocusSyncNP, nullptr);
}

int AstrofocusFocuser::stringToInt(const char *str, bool * has_errors)
{
    int ret = 0;

    try
    {
        ret = std::stoi(str);
        *has_errors = false;

        DEBUGF(INDI::Logger::DBG_DEBUG, "AstrofocusFocuser::stringToInt => str: %s converted to %d.", str, ret);
    }
    catch (std::invalid_argument const &e)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocuser::stringToInt => invalid_argument: %s", e.what());
        ret = 0;
        *has_errors = true;
    }
    catch (std::out_of_range const &e)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocuser::stringToInt => out_of_range: %s", e.what());
        ret = 0;
        *has_errors = true;
    }

    return ret;
}

float AstrofocusFocuser::stringToFloat(const char *str, bool * has_errors)
{
    float ret = 0;

    try
    {
        ret = std::stof(str);
        *has_errors = false;

        DEBUGF(INDI::Logger::DBG_DEBUG, "AstrofocusFocuser::stringToInt => str: %s converted to %f.", str, ret);
    }
    catch (std::invalid_argument const &e)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocuser::stringToFloat => invalid_argument: %s", e.what());
        ret = 0;
        *has_errors = true;
    }
    catch (std::out_of_range const &e)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocuser::stringToFloat => out_of_range: %s", e.what());
        ret = 0;
        *has_errors = true;
    }

    return ret;
}
