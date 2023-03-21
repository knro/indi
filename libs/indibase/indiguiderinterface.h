/*
    Guider Interface
    Copyright (C) 2011 Jasem Mutlaq (mutlaqja@ikarustech.com)

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
#include <functional>

/**
 * @class GuiderInterface
 * @brief Provides interface to implement guider (ST4) port functionality.
 *
 * The child class implements GuideXXXX() functions and returns:
 * IPS_OK if the guide operation is completed in the function, which is usually appropriate for
 * very short guiding pulses.
 * IPS_BUSY if the guide operation is in progress and will take time to complete. In this
 * case, the child class must call GuideComplete() once the guiding pulse is complete.
 * IPS_ALERT if the guide operation failed.
 *
 * \e IMPORTANT: initGuiderProperties() must be called before any other function to initialize
 * the guider properties.
 * \e IMPORATNT: processGuiderProperties() must be called in your driver's ISNewNumber(..)
 * function. processGuiderProperties() will call the guide functions
 * GuideXXXX functions according to the driver.
 *
 * @author Jasem Mutlaq
 */

#include <stdint.h>
#include "abstractinterface.h"

namespace INDI
{

class GuiderInterface : public AbstractInterface
{
    public:

        explicit GuiderInterface(DefaultDevice *device,
                                 std::function<IPState(INDI_DIR_NS, uint32_t)> &ns,
                                 std::function<IPState(INDI_DIR_WE, uint32_t)> &we);
        ~GuiderInterface() = default;

        /**
         * @brief Call GuideComplete once the guiding pulse is complete.
         * @param axis Axis of completed guiding operation.
         */
        void setGuideComplete(INDI_EQ_AXIS axis);

        /**
         * @brief Initilize guider properties. It is recommended to call this function within
         * initProperties() of your primary device
         * @param group Group or tab name to be used to define guider properties.
         */
        void initProperties(const char *group) override;

        /**
         * @brief Call this function whenever client updates GuideNSNP or GuideWSP properties in the
         * primary device. This function then takes care of issuing the corresponding GuideXXXX
         * function accordingly.
         * @param name device name
         * @param values value as passed by the client
         * @param names names as passed by the client
         * @param n number of values and names pair to process.
         */
        bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;

        INDI::PropertyNumber GuideNSNP {2};
        INDI::PropertyNumber GuideWENP {2};

        std::function<IPState(INDI_DIR_NS, uint32_t)> m_GuideNSFP;
        std::function<IPState(INDI_DIR_WE, uint32_t)> m_GuideWEFP;
};
}
