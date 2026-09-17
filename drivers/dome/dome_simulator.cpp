/*******************************************************************************
 Dome Simulator
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
#include "dome_simulator.h"

#include "indicom.h"

#include <cmath>
#include <memory>
#include <algorithm>
#include <unistd.h>

// We declare an auto pointer to domeSim.
static std::unique_ptr<DomeSim> domeSim(new DomeSim());

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
DomeSim::DomeSim()
{
    targetAz        = 0;
    prev_az         = 0;
    prev_alt        = 0;

    SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_ABS_MOVE | DOME_CAN_REL_MOVE | DOME_CAN_PARK | DOME_HAS_SHUTTER);
    SetDomeMorphology(MORPH_ROTATING);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DomeSim::initProperties()
{
    // The morphology must be known before the base class initializes its shutter properties
    // so it can size them (single shutter vs multi-leaf + partial aperture).
    MorphologySP[MORPH_SIM_STANDARD].fill("DOME_MORPHOLOGY_STANDARD", "Standard", ISS_ON);
    MorphologySP[MORPH_SIM_CLAMSHELL].fill("DOME_MORPHOLOGY_CLAMSHELL", "Clamshell", ISS_OFF);
    MorphologySP.fill(getDeviceName(), "DOME_MORPHOLOGY", "Morphology", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 60,
                      IPS_IDLE);
    MorphologySP.load();

    m_Clamshell = MorphologySP[MORPH_SIM_CLAMSHELL].getState() == ISS_ON;
    if (m_Clamshell)
    {
        SetDomeMorphology(MORPH_CLAMSHELL);
        SetShutterCount(2);
        capability |= DOME_CAN_PARTIAL_SHUTTER;
    }
    else
    {
        SetDomeMorphology(MORPH_ROTATING);
    }

    INDI::Dome::initProperties();

    const int count = getShutterCount();
    m_LeafAperture.assign(count, 0);
    m_LeafTarget.assign(count, 0);

    SpeedNP[Dome].fill("DOME", "Dome (deg/s)", "%.2f", 0.1, 10, 1, 5);
    SpeedNP[Shutter].fill("SHUTTER", "Shutter (m/s)", "%.2f", 0.01, 1, 0.1, 0.1);
    SpeedNP.fill(getDeviceName(), "SPEEDS", "Speeds", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    SetParkDataType(PARK_AZ);

    addAuxControls();

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DomeSim::SetupParms()
{
    targetAz     = 0;
    DomeAbsPosNP[0].setValue(0);
    DomeAbsPosNP.apply();

    if (InitPark())
    {
        // If loading parking data is successful, we just set the default parking values.
        SetAxis1ParkDefault(90);
    }
    else
    {
        // Otherwise, we set all parking data to default in case no parking data is found.
        SetAxis1Park(90);
        SetAxis1ParkDefault(90);
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
const char *DomeSim::getDefaultName()
{
    return "Dome Simulator";
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DomeSim::updateProperties()
{
    INDI::Dome::updateProperties();

    if (isConnected())
    {
        defineProperty(MorphologySP);
        defineProperty(SpeedNP);
        SetupParms();
    }
    else
    {
        deleteProperty(MorphologySP);
        deleteProperty(SpeedNP);
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DomeSim::Connect()
{
    SetTimer(1000);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DomeSim::Disconnect()
{
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DomeSim::ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (SpeedNP.isNameMatch(name))
        {
            SpeedNP.update(values, names, n);
            SpeedNP.setState(IPS_OK);
            SpeedNP.apply();
            saveConfig(SpeedNP);
            return true;
        }
    }

    return INDI::Dome::ISNewNumber(dev, name, values, names, n);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DomeSim::ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (MorphologySP.isNameMatch(name))
        {
            MorphologySP.update(states, names, n);
            MorphologySP.setState(IPS_OK);
            MorphologySP.apply();
            saveConfig(MorphologySP);
            LOG_INFO("Driver must be restarted for this change to take effect.");
            return true;
        }
    }

    return INDI::Dome::ISNewSwitch(dev, name, states, names, n);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
void DomeSim::TimerHit()
{
    if (!isConnected())
        return;

    auto domeSpeed = SpeedNP[Dome].getValue();
    auto shutterSpeed = SpeedNP[Shutter].getValue();

    if (DomeAbsPosNP.getState() == IPS_BUSY)
    {
        double b = DomeAbsPosNP[0].getValue();

        if (m_MoveDirection != 0)
        {
            // Continuous jog: DOME_CW increases azimuth, DOME_CCW decreases it.
            DomeAbsPosNP[0].setValue(range360(b + domeSpeed * m_MoveDirection));
        }
        else
        {
            // Find shortest distance given target degree
            double a = targetAz;
            int sign = (a - b >= 0 && a - b <= 180) || (a - b <= -180 && a - b >= -360) ? 1 : -1;
            DomeAbsPosNP[0].setValue(range360(b + domeSpeed * sign));

            if (std::abs(targetAz - DomeAbsPosNP[0].getValue()) <= domeSpeed)
            {
                DomeAbsPosNP[0].setValue(targetAz);
                LOG_INFO("Dome reached requested azimuth angle.");

                if (getDomeState() == DOME_PARKING)
                    SetParked(true);
                else if (getDomeState() == DOME_UNPARKING)
                    SetParked(false);
                else
                    setDomeState(DOME_SYNCED);
            }
        }

        DomeAbsPosNP.apply();
    }

    // Dome shutter, we decrease by shutter speed in meters/second
    if (DomeShutterSP.getState() == IPS_BUSY)
    {
        m_ShutterDistance -= shutterSpeed;

        if (m_ShutterDistance <= 0)
        {
            m_ShutterDistance = 0;
            DomeShutterSP.setState(IPS_OK);
            LOGF_INFO("Shutter is %s.", (DomeShutterSP[0].getState() == ISS_ON ? "open" : "closed"));
            DomeShutterSP.apply();

            if (getDomeState() == DOME_UNPARKING)
                SetParked(false);
        }
    }

    // Clamshell: advance each leaf toward its target (10% per poll) and report the live
    // aperture in DOME_APERTURE.
    if (m_Clamshell && HasShutter())
    {
        bool changed = false;
        for (int i = 0; i < getShutterCount(); i++)
        {
            if (getShutterState(i) != SHUTTER_MOVING)
                continue;

            const double target = m_LeafTarget[i];
            const double step   = 10.0;
            double ap           = m_LeafAperture[i];

            if (std::abs(target - ap) <= step)
            {
                ap = target;
                setShutterState(i, ap > 0 ? SHUTTER_OPENED : SHUTTER_CLOSED);
                LOGF_INFO("Shutter %d reached %.0f%% aperture.", i + 1, ap);
            }
            else
                ap += (target > ap ? step : -step);

            m_LeafAperture[i] = ap;
            changed           = true;
        }

        if (changed)
        {
            for (int i = 0; i < getShutterCount(); i++)
                DomeApertureNP[i].setValue(m_LeafAperture[i]);

            bool anyMoving = false;
            for (int i = 0; i < getShutterCount(); i++)
                anyMoving = anyMoving || (getShutterState(i) == SHUTTER_MOVING);

            DomeApertureNP.setState(anyMoving ? IPS_BUSY : IPS_OK);
            DomeApertureNP.apply();
        }
    }

    SetTimer(getPollingPeriod());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
IPState DomeSim::Move(DomeDirection dir, DomeMotionCommand operation)
{
    if (operation == MOTION_START)
    {
        // DOME_CW increases the azimuth, DOME_CCW decreases it (see INDI::Dome convention).
        m_MoveDirection = (dir == DOME_CW) ? 1 : -1;
        DomeAbsPosNP.setState(IPS_BUSY);
    }
    else
    {
        m_MoveDirection = 0;
        targetAz        = DomeAbsPosNP[0].getValue();
        DomeAbsPosNP.setState(IPS_IDLE);
    }

    DomeAbsPosNP.apply();
    return ((operation == MOTION_START) ? IPS_BUSY : IPS_OK);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
IPState DomeSim::MoveAbs(double az)
{
    m_MoveDirection = 0;
    targetAz = az;

    // Requested position is within one cycle, let's declare it done
    if (std::abs(az - DomeAbsPosNP[0].getValue()) < SpeedNP[Dome].getValue())
        return IPS_OK;

    // It will take a few cycles to reach final position
    return IPS_BUSY;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
IPState DomeSim::MoveRel(double azDiff)
{
    m_MoveDirection = 0;
    targetAz = range360(DomeAbsPosNP[0].getValue() + azDiff);

    // Requested position is within one cycle, let's declare it done
    if (std::abs(targetAz - DomeAbsPosNP[0].value) < SpeedNP[Dome].getValue())
        return IPS_OK;

    // It will take a few cycles to reach final position
    return IPS_BUSY;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
IPState DomeSim::Park()
{
    targetAz = DomeParamNP[0].getValue();
    Dome::ControlShutter(SHUTTER_CLOSE);
    Dome::MoveAbs(GetAxis1Park());

    return IPS_BUSY;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
IPState DomeSim::UnPark()
{
    return Dome::ControlShutter(SHUTTER_OPEN);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
IPState DomeSim::ControlShutter(ShutterOperation operation)
{
    INDI_UNUSED(operation);
    // If shutter width = 1 meter, then we only need to open half of that since we presume
    // two sides are moving at the same speed closing or opening.
    m_ShutterDistance = DomeMeasurementsNP[DM_SHUTTER_WIDTH].getValue() / 2;
    return IPS_BUSY;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
IPState DomeSim::SetAperture(int index, double percent)
{
    if (index < 0 || index >= getShutterCount())
        return IPS_ALERT;

    percent = std::max(0.0, std::min(100.0, percent));

    if (std::abs(percent - m_LeafAperture[index]) < 1)
    {
        setShutterState(index, percent > 0 ? SHUTTER_OPENED : SHUTTER_CLOSED);
        return IPS_OK;
    }

    m_LeafTarget[index]    = percent;
    setShutterState(index, SHUTTER_MOVING);
    return IPS_BUSY;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
double DomeSim::GetAperture(int index) const
{
    if (index < 0 || index >= static_cast<int>(m_LeafAperture.size()))
        return -1;
    return m_LeafAperture[index];
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DomeSim::Abort()
{
    // If we abort while in the middle of opening/closing shutter, alert.
    if (DomeShutterSP.getState() == IPS_BUSY)
    {
        DomeShutterSP.setState(IPS_ALERT);
        LOG_ERROR("Shutter operation aborted. Status: unknown.");
        DomeShutterSP.apply();
        return false;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DomeSim::SetCurrentPark()
{
    SetAxis1Park(DomeAbsPosNP[0].getValue());
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DomeSim::SetDefaultPark()
{
    // By default set position to 90
    SetAxis1Park(90);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////////////////////////
bool DomeSim::saveConfigItems(FILE * fp)
{
    Dome::saveConfigItems(fp);
    MorphologySP.save(fp);
    SpeedNP.save(fp);
    return true;
}