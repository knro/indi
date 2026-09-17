/*******************************************************************************
  Roll-Off Simulator
  Copyright(c) 2014 Jasem Mutlaq. All rights reserved.

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

#include "roll_off.h"

#include "indicom.h"

#include <cmath>
#include <cstring>
#include <ctime>
#include <memory>

// We declare an auto pointer to RollOff.
std::unique_ptr<RollOff> rollOff(new RollOff());

namespace
{
constexpr double ROLLOFF_DURATION = 10; // 10 seconds until roof is fully opened or closed
}

RollOff::RollOff()
{
    setVersion(1, 0);
}

const char *RollOff::getDefaultName()
{
    return "RollOff Simulator";
}

bool RollOff::initProperties()
{
    INDI::RollOffDome::initProperties();
    addAuxControls();
    setDefaultPollingPeriod(1000);
    return true;
}

bool RollOff::Connect()
{
    SetTimer(getCurrentPollingPeriod());
    return true;
}

bool RollOff::Disconnect()
{
    return true;
}

bool RollOff::saveConfigItems(FILE *fp)
{
    return INDI::RollOffDome::saveConfigItems(fp);
}

////////////////////////////////////////////////////////////////////////////////////////
/// Hardware interface
////////////////////////////////////////////////////////////////////////////////////////
bool RollOff::startOpenRoof()
{
    if (isFullyOpen())
    {
        LOG_WARN("Roof is already fully opened.");
        return false;
    }

    fullOpenLimitSwitch   = ISS_OFF;
    fullClosedLimitSwitch = ISS_OFF;
    MotionRequest         = ROLLOFF_DURATION;
    gettimeofday(&MotionStart, nullptr);
    LOG_INFO("Roll off is opening...");
    return true;
}

bool RollOff::startCloseRoof()
{
    if (isFullyClosed())
    {
        LOG_WARN("Roof is already fully closed.");
        return false;
    }

    fullOpenLimitSwitch   = ISS_OFF;
    fullClosedLimitSwitch = ISS_OFF;
    MotionRequest         = ROLLOFF_DURATION;
    gettimeofday(&MotionStart, nullptr);
    LOG_INFO("Roll off is closing...");
    return true;
}

bool RollOff::stopRoof()
{
    return true;
}

bool RollOff::isFullyOpen()
{
    return fullOpenLimitSwitch == ISS_ON;
}

bool RollOff::isFullyClosed()
{
    return fullClosedLimitSwitch == ISS_ON;
}

void RollOff::updateRoof()
{
    if (getRoofStatus() == ROOF_IDLE)
        return;

    // The simulator uses a timed motion to emulate the roof reaching a limit switch.
    if (CalcTimeLeft(MotionStart) <= 0)
    {
        if (getRoofStatus() == ROOF_OPENING)
            fullOpenLimitSwitch = ISS_ON;
        else
            fullClosedLimitSwitch = ISS_ON;

        MotionRequest = 0;
    }
}

float RollOff::CalcTimeLeft(timeval start)
{
    double timesince;
    double timeleft;
    struct timeval now
    {
        0, 0
    };
    gettimeofday(&now, nullptr);

    timesince =
        (double)(now.tv_sec * 1000.0 + now.tv_usec / 1000) - (double)(start.tv_sec * 1000.0 + start.tv_usec / 1000);
    timesince = timesince / 1000;
    timeleft  = MotionRequest - timesince;
    return timeleft;
}
