/*******************************************************************************
  Copyright(c) 2020 Giacomo Succi. All rights reserved.

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
*******************************************************************************/

#ifndef ASTROFOCUS_FOCUSER_H

    #define ASTROFOCUS_FOCUSER_H

    #include <indifocuser.h>
    #include "config.h"

    #define MESSAGE_MAX_LENGHT  50
    #define READ_TIMEOUT        5

    class AstrofocusFocuser : public INDI::Focuser
    {
        public:
            AstrofocusFocuser();
            ~AstrofocusFocuser();
            
            virtual bool Handshake();
            virtual void ISGetProperties(const char *dev);
            virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        protected:
            const char *getDefaultName();
            bool initProperties() override;
            bool updateProperties() override;

            int sendCommand(const char *cmd);
            bool receivedAck();
            char * receiveResponse();

            void loadSettingsFromDevice();

            int stringToInt(const char *str, bool * has_errors);
            float stringToFloat(const char *str, bool * has_errors);
        private:
            enum
            {
                STEPPER_MODE_ONE_PHASE_FULL_STEP,
                STEPPER_MODE_TWO_PHASE_FULL_STEP,
                STEPPER_MODE_HALF_STEP,
                STEPPER_MODE_COUNT
            };

            ISwitch StepperModeS[STEPPER_MODE_COUNT];
            ISwitchVectorProperty StepperModeSP;

            INumber StepSizeN[1] {};
            INumberVectorProperty StepSizeNP;

            IText FirmwareVersionT[1] {};
            ITextVectorProperty FirmwareVersionTP;
    };
#endif
