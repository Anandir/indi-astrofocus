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


 * Comandi seriali di AstroFocus 5
 * -------------------------------
 * 
 * 0,0	Restituisce posizione attuale
 * 0,N	Imposta la posizione attuale
 * 1,N	Va alla posizione assoluta N
 * 2,N	Va alla posizione relativa N ignorando i limiti
 * 3,0	Imposta la posizione attuale come limite inferiore (punto 0)
 * 4,0	Restituisce il limite superiore attuale
 * 4,1	Imposta la posizione attuale come limite superiore
 * 4,N	Imposta il limite superiore a N (>1) (*)
 * 5,0	Restituisce T se è presente il sensore di temperature, altrimenti F
 * 5,1	Restituisce la temperatura attuale
 * 6,0	Restituisce il coefficiente di temperatura
 * 6,N	Imposta il coefficiente di temperatura a N
 * 7,0	Disattiva la compensazione della temperatura
 * 7,1	Attiva la compensazione della temperatura
 * 8,0	Restituisce la dimensione dei passi in 1/100 di micron
 * 8,N	Imposta la dimensione dei passi in 1/100 di micron
 * 9,0	Restituisce la versione del programma
 * 10,0 Restituisce la potenza del motore (1-255)
 * 10,N Imposta la potenza del motore (1-255)
 * 11,0 Restituisce la durata degli impulsi in millisecondi
 * 11,N Imposta la durata degli impulsi in millisecondi
 * 12,0 Restituisce la pausa prima dell'interruzione dell'alimentazione del motore in millisecondi
 * 12,N Imposta la pausa prima dell'interruzione dell'alimentazione del motore in millisecondi
 * 13,0 Restituisce la modalità di spostamento (1: One Phase Full Step, 2: Two Phase Full Step, 3: Half Step)
 * 13,N Imposta la modalità  di spostamento (1: One Phase Full Step, 2: Two Phase Full Step, 3: Half Step)
 ***********************************************************************************/

#include <cmath>
#include <memory>
#include <cstring>
#include <unistd.h>
#include <termios.h>

#include <indicom.h>
#include "connectionplugins/connectionserial.h"

#include "astrofocus_focuser.h"

static std::unique_ptr<AstrofocusFocus> astrofocusFocus(new AstrofocusFocus());

/**************************************************************************************
 ** Constructor
 ***************************************************************************************/
AstrofocusFocus::AstrofocusFocus()
{
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_HAS_VARIABLE_SPEED | FOCUSER_CAN_SYNC);

    setSupportedConnections(CONNECTION_SERIAL);

    setVersion(AFF_MAJOR_VERSION, AFF_MINOR_VERSION);
}

/**************************************************************************************
 ** Distructor
 ***************************************************************************************/
AstrofocusFocus::~AstrofocusFocus()
{
}

/**************************************************************************************
 ** INDI is asking us for our default device name
 ***************************************************************************************/
const char * AstrofocusFocus::getDefaultName()
{
    return "Astrofocus";
}

/**************************************************************************************
 ** 
 ***************************************************************************************/
bool AstrofocusFocus::initProperties()
{
    INDI::Focuser::initProperties();

    addDebugControl();

    IUFillSwitch(&StepperModeS[STEPPER_MODE_ONE_PHASE_FULL_STEP], "One Phase Full Step", "One Phase Full Step", ISS_ON);
    IUFillSwitch(&StepperModeS[STEPPER_MODE_TWO_PHASE_FULL_STEP], "Two Phase Full Step", "Two Phase Full Step", ISS_OFF);
    IUFillSwitch(&StepperModeS[STEPPER_MODE_HALF_STEP], "Half Step", "Half Step", ISS_OFF);
    IUFillSwitchVector(&StepperModeSP, StepperModeS, STEPPER_MODE_COUNT, getDeviceName(),
                       "STEPPER_MODE", "Stepper Mode", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY,
                       60, IPS_IDLE);
    
    /* Relative and absolute movement */
    FocusRelPosN[0].min = 0.;
    FocusRelPosN[0].max = 0.;
    FocusRelPosN[0].value = 0.;
    FocusRelPosN[0].step = 0.;

    FocusAbsPosN[0].min = 0.;
    FocusAbsPosN[0].max = 32767.;
    FocusAbsPosN[0].value = 0.;
    FocusAbsPosN[0].step = 0.;

    return true;
}

/**************************************************************************************
 ** 
 ***************************************************************************************/
bool AstrofocusFocus::updateProperties()
{
    INDI::Focuser::updateProperties();
    
    if (isConnected())
        defineSwitch(&StepperModeSP);
    else
        deleteProperty(StepperModeSP.name);
    
    return true;
}

/**************************************************************************************
 ** Return properties of device.
 ***************************************************************************************/
void ISGetProperties (const char *dev)
{
    astrofocusFocus->ISGetProperties(dev);
}
 
/**************************************************************************************
 ** Process new switch from client
 ***************************************************************************************/
void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
    astrofocusFocus->ISNewSwitch(dev, name, states, names, num);
}

bool AstrofocusFocus::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if(!strcmp(dev, getDeviceName()))
    {
        if (!strcmp(name, StepperModeSP.name))
        {
            int currentIndex = IUFindOnSwitchIndex(&StepperModeSP);

            switch (currentIndex)
            {
                case STEPPER_MODE_ONE_PHASE_FULL_STEP:
                {
                    sendCommand("13,1\n");
                    break;
                }
                case STEPPER_MODE_TWO_PHASE_FULL_STEP:
                {
                    sendCommand("13,2\n");
                    break;
                }
                case STEPPER_MODE_HALF_STEP:
                {
                    sendCommand("13,3\n");
                    break;
                }
                default:
                {
                    StepperModeSP.s = IPS_ALERT;
                    IDSetSwitch(&StepperModeSP, "AstrofocusFocus::ISNewSwitch => Unknown mode index %d", currentIndex);
                    return true;
                }
            }

            
            if(!receivedAck())
            {
                StepperModeSP.s = IPS_ALERT;
                IDSetSwitch(&StepperModeSP, "AstrofocusFocus::ISNewSwitch => Ack not received for index %d", currentIndex);
                return false;
            }

            IUUpdateSwitch(&StepperModeSP, states, names, n);
            currentIndex = IUFindOnSwitchIndex(&StepperModeSP);
            StepperModeSP.s = IPS_OK;
            IDSetSwitch(&StepperModeSP, NULL);

            DEBUGF(INDI::Logger::DBG_SESSION, "AstrofocusFocus::ISNewSwitch => The new value is %s", StepperModeS[currentIndex].label);
            
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
    astrofocusFocus->ISNewText(dev, name, texts, names, n);
}
 
/**************************************************************************************
 ** Process new number from client
 ***************************************************************************************/
void ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
    astrofocusFocus->ISNewNumber(dev, name, values, names, n);
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
    astrofocusFocus->ISSnoopDevice(root);
}

/* ************************************************************************************ */

bool AstrofocusFocus::Handshake()
{
    int nbytes_read = 0, err_code = 0;
    char response[MESSAGE_MAX_LENGHT];
    char error_message[MAXRBUF];

    tcflush(PortFD, TCIOFLUSH);

    DEBUGF(INDI::Logger::DBG_DEBUG, "AstrofocusFocus::Handshake => PortFD: %d", PortFD);

    sendCommand("9,0\n");

    if ((err_code = tty_read_section(PortFD, response, '\n', READ_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(err_code, error_message, MAXRBUF);

        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocus::Handshake => TTY read error detected: %s", error_message);

        return false;
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "AstrofocusFocus::Handshake => Response: %s", response);

    return true;
}

/**************************************************************************************
 ** Serial communications
 ***************************************************************************************/
int AstrofocusFocus::sendCommand(const char *cmd)
{
    int nbytes_written = 0, err_code = 0;
    char err_msg[MAXRBUF];

    tcflush(PortFD, TCIOFLUSH);

    if ((err_code = tty_write_string(PortFD, cmd, &nbytes_written) != TTY_OK))
    {
        tty_error_msg(err_code, err_msg, MAXRBUF);

        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocus::sendCommand => cmd: %s", cmd);
        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocus::sendCommand => TTY error detected: %s", err_msg);
        return -1;
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "AstrofocusFocus::sendCommand => Command sent successfully: %s", cmd);

    return nbytes_written;
}

/* ************************************************************************************ */

bool AstrofocusFocus::receivedAck()
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

char * AstrofocusFocus::receiveResponse()
{
    int nbytes_read = 0, err_code = 0;
    char error_message[MAXRBUF];
    char *response = (char *)calloc(MESSAGE_MAX_LENGHT, sizeof(char));

    tcflush(PortFD, TCIOFLUSH);

    if ((err_code = tty_read_section(PortFD, response, '\n', READ_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(err_code, error_message, MAXRBUF);

        DEBUGF(INDI::Logger::DBG_ERROR, "AstrofocusFocus::receiveResult => TTY read error detected: %s", error_message);

        return NULL;
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "AstrofocusFocus::receiveResult => Response: %s", response);

    return response;
}
