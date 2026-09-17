/*******************************************************************************
  INDI Roll-off Roof Base Class
  Copyright(c) 2024 Jasem Mutlaq. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Library General Public
  License version 2 as published by the Free Software Foundation.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Library General Public License for more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; see the file COPYING.LIB.  If not, write to
  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
  Boston, MA 02110-1301, USA.
*******************************************************************************/

#include "indidomerolloff.h"

namespace INDI
{

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
RollOffDome::RollOffDome()
{
    SetDomeMorphology(MORPH_ROLLOFF);
    SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_PARK);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool RollOffDome::initProperties()
{
    if (!Dome::initProperties())
        return false;

    // A roll-off roof has no azimuth axis, so there is no stored park position.
    SetParkDataType(PARK_NONE);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
IPState RollOffDome::Move(DomeDirection dir, DomeMotionCommand operation)
{
    if (operation == MOTION_START)
    {
        // DOME_CW --> OPEN. If we are asked to "open" while the fully-open limit is asserted, reject.
        if (dir == DOME_CW && isFullyOpen())
        {
            LOG_WARN("Roof is already fully opened.");
            return IPS_ALERT;
        }
        else if (dir == DOME_CCW && isFullyClosed())
        {
            LOG_WARN("Roof is already fully closed.");
            return IPS_ALERT;
        }
        else if (dir == DOME_CCW && Dome::isLocked())
        {
            LOG_WARN("Cannot close dome when mount is locking. See: Telescope parking policy, in options tab");
            return IPS_ALERT;
        }

        const bool accepted = (dir == DOME_CW) ? startOpenRoof() : startCloseRoof();
        if (!accepted)
        {
            LOG_ERROR("Failed to start roof motion.");
            return IPS_ALERT;
        }

        m_RoofStatus  = (dir == DOME_CW) ? ROOF_OPENING : ROOF_CLOSING;
        m_RoofAborted = false;
        return IPS_BUSY;
    }

    // MOTION_STOP
    return stopRoof() ? IPS_OK : IPS_ALERT;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
IPState RollOffDome::Park()
{
    return Dome::Move(DOME_CCW, MOTION_START);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
IPState RollOffDome::UnPark()
{
    return Dome::Move(DOME_CW, MOTION_START);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool RollOffDome::Abort()
{
    m_RoofAborted = true;
    const bool rc = stopRoof();
    m_RoofStatus  = ROOF_IDLE;

    // If neither limit switch is asserted, then we are neither parked nor unparked.
    if (!isFullyOpen() && !isFullyClosed())
    {
        ParkSP.reset();
        ParkSP.setState(IPS_IDLE);
        ParkSP.apply();
    }

    return rc;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
void RollOffDome::TimerHit()
{
    if (!isConnected())
        return;

    updateRoof();

    if (m_RoofStatus != ROOF_IDLE)
    {
        if (m_RoofStatus == ROOF_OPENING && isFullyOpen())
        {
            m_RoofStatus = ROOF_IDLE;
            setDomeState(DOME_IDLE);
            SetParked(false);
            LOG_INFO("Roof is fully opened.");
        }
        else if (m_RoofStatus == ROOF_CLOSING && isFullyClosed())
        {
            m_RoofStatus = ROOF_IDLE;
            setDomeState(DOME_IDLE);
            SetParked(true);
            LOG_INFO("Roof is fully closed.");
        }
    }

    SetTimer(getCurrentPollingPeriod());
}

}
