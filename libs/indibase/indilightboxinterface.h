/*
    Light Box / Switch Interface
    Copyright (C) 2015 Jasem Mutlaq (mutlaqja@ikarustech.com)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#pragma once

#include "abstractinterface.h"

#include <stdint.h>

/**
 * \class LightBoxInterface
   \brief Provides interface to implement controllable light box/switch device.

   Filter durations preset can be defined if the active filter name is set. Once the filter names are retrieved, the duration in seconds can be set for each filter.
   When the filter wheel changes to a new filter, the duration is set accordingly.

   Devices implementing LightBox interface need to implement callback functions for setting the brightness level and toggling
   the light.
\author Jasem Mutlaq
*/
namespace INDI
{

class LightBoxInterface : public AbstractInterface
{
    protected:
        explicit LightBoxInterface(DefaultDevice *device);
        virtual ~LightBoxInterface();


        /**
             * @brief registerSetLightBoxEnabled Set light level. Must be impelemented in the child class, if supported.
             * @param value level of light box
             * @return True if successful, false otherwise.
             */
        void registerSetLightBoxEnabled(const std::function<bool(bool)> &cb)
        {
            m_SetLightBoxEnabled = cb;
        }

        /**
             * @brief SetLightBoxEnabled Turn on/off on a light box. Must be impelemented in the child class.
             * @param enable If true, turn on the light, otherwise turn off the light.
             * @return True if successful, false otherwise.
             */
        void registerSetLightBoxIntensity(const std::function<bool(uint32_t)> &cb)
        {
            m_SetLightBoxIntensity = cb;
        }

        /** \brief Initilize light box properties. It is recommended to call this function within initProperties() of your primary device
                \param deviceName Name of the primary device
                \param groupName Group or tab name to be used to define light box properties.
            */
        void initProperties(const char *group) override;

        /**
             * @brief isGetLightBoxProperties Get light box properties
             * @param deviceName parent device name
             */
        void ISGetProperties(const char *deviceName) override;

        /** \brief Define or delete light properties depending on connection */
        bool updateProperties() override;

        /** \brief Process light box switch properties */
        bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;

        /** \brief Process light box number properties */
        bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;

        /** \brief Process light box text properties */
        bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;

        bool saveConfigItems(FILE *fp) override;
        bool ISSnoopDevice(XMLEle *root) override;

        // Turn on/off light
        INDI::PropertySwitch LightSP {2};

        // Light Intensity
        INDI::PropertyNumber LightIntensityNP {1};

        // Active devices to snoop
        INDI::PropertyText ActiveDeviceTP {1};

        INDI::PropertyNumber FilterIntensityNP {0};

    private:
        void addFilterDuration(const char *filterName, uint16_t filterDuration);

        std::function<bool(uint16_t)> m_SetLightBoxIntensity;
        std::function<bool(bool)> m_SetLightBoxEnabled;

        uint8_t m_CurrentFilterSlot {0};
        bool m_isDimmable;
        char m_ConfigFilter[MAXINDIDEVICE];
};
}
