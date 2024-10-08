/*
    Astro-Electronic FS-2 Driver
    Copyright (C) 2015-2023 Jasem Mutlaq (mutlaqja@ikarustech.com)

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

#include "lx200fs2.h"

#include "indicom.h"
#include "lx200driver.h"

#include <libnova/transform.h>

#include <cmath>
#include <cstring>

LX200FS2::LX200FS2() : LX200Generic()
{
    setVersion(2, 3);

    SetTelescopeCapability(
        TELESCOPE_CAN_PARK | TELESCOPE_CAN_SYNC | TELESCOPE_CAN_GOTO | TELESCOPE_HAS_LOCATION | TELESCOPE_CAN_ABORT, 4);
}

bool LX200FS2::initProperties()
{
    LX200Generic::initProperties();

    IUFillNumber(&SlewAccuracyN[0], "SlewRA", "RA (arcmin)", "%10.6m", 0., 60., 1., 3.0);
    IUFillNumber(&SlewAccuracyN[1], "SlewDEC", "Dec (arcmin)", "%10.6m", 0., 60., 1., 3.0);
    IUFillNumberVector(&SlewAccuracyNP, SlewAccuracyN, NARRAY(SlewAccuracyN), getDeviceName(), "Slew Accuracy", "",
                       OPTIONS_TAB, IP_RW, 0, IPS_IDLE);
    IUFillSwitchVector(&StopAfterParkSP, StopAfterParkS, 2, getDeviceName(), "Stop after Park", "Stop after Park", OPTIONS_TAB,
                       IP_RW, ISR_ATMOST1, 0, IPS_IDLE);
    IUFillSwitch(&StopAfterParkS[0], "ON", "ON", ISS_OFF);
    IUFillSwitch(&StopAfterParkS[1], "OFF", "OFF", ISS_ON);

    SetParkDataType(PARK_AZ_ALT);

    return true;
}

bool LX200FS2::updateProperties()
{
    LX200Generic::updateProperties();

    if (isConnected())
    {
        defineProperty(SlewRateSP);
        defineProperty(&SlewAccuracyNP);
        defineProperty(&StopAfterParkSP);

        if (InitPark())
        {
            // If loading parking data is successful, we just set the default parking values.
            SetAxis1ParkDefault(0);
            SetAxis2ParkDefault(LocationNP[LOCATION_LATITUDE].getValue());

            if (isParked())
            {
                // Force tracking to stop at startup.
                ParkedStatus = PARKED_NOTPARKED;
                TrackingStop();
            }
        }
        else
        {
            // Otherwise, we set all parking data to default in case no parking data is found.
            SetAxis1Park(0);
            SetAxis2Park(LocationNP[LOCATION_LATITUDE].getValue());
            SetAxis1ParkDefault(0);
            SetAxis2ParkDefault(LocationNP[LOCATION_LATITUDE].getValue());
        }
    }
    else
    {
        deleteProperty(SlewRateSP);
        deleteProperty(SlewAccuracyNP.name);
        deleteProperty(StopAfterParkSP.name);
    }

    return true;
}

bool LX200FS2::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (!strcmp(name, SlewAccuracyNP.name))
        {
            if (IUUpdateNumber(&SlewAccuracyNP, values, names, n) < 0)
                return false;

            SlewAccuracyNP.s = IPS_OK;

            if (SlewAccuracyN[0].value < 3 || SlewAccuracyN[1].value < 3)
                IDSetNumber(&SlewAccuracyNP, "Warning: Setting the slew accuracy too low may result in a dead lock");

            IDSetNumber(&SlewAccuracyNP, nullptr);
            return true;
        }
    }

    return LX200Generic::ISNewNumber(dev, name, values, names, n);
}

bool LX200FS2::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (!strcmp(name, StopAfterParkSP.name))
        {
            // Find out which state is requested by the client
            const char *actionName = IUFindOnSwitchName(states, names, n);
            // If switch is the same state as actionName, then we do nothing.
            int currentIndex = IUFindOnSwitchIndex(&StopAfterParkSP);
            if (!strcmp(actionName, StopAfterParkS[currentIndex].name))
            {
                DEBUGF(INDI::Logger::DBG_SESSION, "Stop After Park is already %s", StopAfterParkS[currentIndex].label);
                StopAfterParkSP.s = IPS_IDLE;
                IDSetSwitch(&StopAfterParkSP, NULL);
                return true;
            }

            // Otherwise, let us update the switch state
            IUUpdateSwitch(&StopAfterParkSP, states, names, n);
            currentIndex = IUFindOnSwitchIndex(&StopAfterParkSP);
            DEBUGF(INDI::Logger::DBG_SESSION, "Stop After Park is now %s", StopAfterParkS[currentIndex].label);
            StopAfterParkSP.s = IPS_OK;
            IDSetSwitch(&StopAfterParkSP, NULL);
            return true;
        }
    }

    return LX200Generic::ISNewSwitch(dev, name, states, names, n);
}

const char *LX200FS2::getDefaultName()
{
    return "Astro-Electronic FS-2";
}

bool LX200FS2::isSlewComplete()
{
    const double dx = targetRA - currentRA;
    const double dy = targetDEC - currentDEC;
    return fabs(dx) <= (SlewAccuracyN[0].value / (900.0)) && fabs(dy) <= (SlewAccuracyN[1].value / 60.0);
}

bool LX200FS2::checkConnection()
{
    return true;
}

bool LX200FS2::saveConfigItems(FILE *fp)
{
    INDI::Telescope::saveConfigItems(fp);

    IUSaveConfigNumber(fp, &SlewAccuracyNP);
    IUSaveConfigSwitch(fp, &StopAfterParkSP);

    return true;
}

bool LX200FS2::Park()
{
    double parkAz  = GetAxis1Park();
    double parkAlt = GetAxis2Park();

    char AzStr[16], AltStr[16];
    fs_sexa(AzStr, parkAz, 2, 3600);
    fs_sexa(AltStr, parkAlt, 2, 3600);
    LOGF_DEBUG("Parking to Az (%s) Alt (%s)...", AzStr, AltStr);

    INDI::IEquatorialCoordinates equatorialCoords {0, 0};
    INDI::IHorizontalCoordinates horizontalCoords {parkAz, parkAlt};
    INDI::HorizontalToEquatorial(&horizontalCoords, &m_Location, ln_get_julian_from_sys(), &equatorialCoords);

    char RAStr[16], DEStr[16];
    fs_sexa(RAStr, equatorialCoords.rightascension, 2, 3600);
    fs_sexa(DEStr, equatorialCoords.declination, 2, 3600);
    LOGF_DEBUG("Parking to RA (%s) DEC (%s)...", RAStr, DEStr);

    if (Goto(equatorialCoords.rightascension, equatorialCoords.declination))
    {
        TrackState = SCOPE_PARKING;
        LOG_INFO("Parking is in progress...");

        return true;
    }
    else
        return false;
}

void LX200FS2::TrackingStop()
{
    if (ParkedStatus != PARKED_NOTPARKED) return;

    // Remember current slew rate
    savedSlewRateIndex = static_cast <enum TelescopeSlewRate> (SlewRateSP.findOnSwitchIndex());

    updateSlewRate(SLEW_CENTERING);
    ParkedStatus = PARKED_NEEDABORT;
}

void LX200FS2::TrackingStop_Abort()
{
    if (ParkedStatus != PARKED_NEEDABORT) return;

    Abort();
    ParkedStatus = PARKED_NEEDSTOP;
}

void LX200FS2::TrackingStop_AllStop()
{
    if (ParkedStatus != PARKED_NEEDSTOP) return;

    MoveWE(DIRECTION_EAST, MOTION_START);
    ParkedStatus = PARKED_STOPPED;
}

void LX200FS2::TrackingStart()
{
    if (ParkedStatus != PARKED_STOPPED) return;

    MoveWE(DIRECTION_EAST, MOTION_STOP);

    ParkedStatus = UNPARKED_NEEDSLEW;
}

void LX200FS2::TrackingStart_RestoreSlewRate()
{
    if (ParkedStatus != UNPARKED_NEEDSLEW) return;

    updateSlewRate(savedSlewRateIndex);

    ParkedStatus = PARKED_NOTPARKED;
}

bool LX200FS2::ReadScopeStatus()
{
    bool retval = LX200Generic::ReadScopeStatus();

    // For FS-2 v1.21 owners, stop tracking once Parked.
    if (retval &&
            StopAfterParkS[0].s == ISS_ON &&
            isConnected() &&
            !isSimulation())
    {
        switch (TrackState)
        {
            case SCOPE_PARKED:
                // If you are changing state from parking to parked,
                // kick off the motor-stopping state machine
                switch (ParkedStatus)
                {
                    case PARKED_NOTPARKED:
                        LOG_INFO("Mount at park position. Tracking stopping.");
                        TrackingStop();
                        break;
                    case PARKED_NEEDABORT:
                        LOG_INFO("Mount at 1x sidereal.");
                        TrackingStop_Abort();
                        break;
                    case PARKED_NEEDSTOP:
                        LOG_INFO("Mount is parked, motors stopped.");
                        TrackingStop_AllStop();
                        break;
                    case PARKED_STOPPED:
                    default:
                        break;

                }
                break;
            case SCOPE_IDLE:
                // If you are changing state from parked to tracking,
                // kick off the motor-starting state machine
                switch (ParkedStatus)
                {
                    case UNPARKED_NEEDSLEW:
                        LOG_INFO("Mount is unparked, restoring slew rate.");
                        TrackingStart_RestoreSlewRate();
                        break;
                    default:
                        break;

                }
                break;
            default:
                break;
        }
        return true;
    }


    return retval;
}

bool LX200FS2::UnPark()
{
    double parkAz  = GetAxis1Park();
    double parkAlt = GetAxis2Park();

    char AzStr[16], AltStr[16];
    fs_sexa(AzStr, parkAz, 2, 3600);
    fs_sexa(AltStr, parkAlt, 2, 3600);
    LOGF_DEBUG("Unparking from Az (%s) Alt (%s)...", AzStr, AltStr);

    INDI::IEquatorialCoordinates equatorialCoords {0, 0};
    INDI::IHorizontalCoordinates horizontalCoords {parkAz, parkAlt};
    INDI::HorizontalToEquatorial(&horizontalCoords, &m_Location, ln_get_julian_from_sys(), &equatorialCoords);

    char RAStr[16], DEStr[16];
    fs_sexa(RAStr, equatorialCoords.rightascension, 2, 3600);
    fs_sexa(DEStr, equatorialCoords.declination, 2, 3600);
    LOGF_DEBUG("Syncing to parked coordinates RA (%s) DEC (%s)...", RAStr, DEStr);

    if (Sync(equatorialCoords.rightascension, equatorialCoords.declination))
    {
        SetParked(false);
        if (StopAfterParkS[0].s == ISS_ON)
        {
            TrackingStart();
        }
        return true;
    }
    else
        return false;
}

bool LX200FS2::SetCurrentPark()
{
    INDI::IEquatorialCoordinates equatorialCoords {currentRA, currentDEC};
    INDI::IHorizontalCoordinates horizontalCoords {0, 0};
    INDI::EquatorialToHorizontal(&equatorialCoords, &m_Location, ln_get_julian_from_sys(), &horizontalCoords);
    double parkAZ = horizontalCoords.azimuth;
    double parkAlt = horizontalCoords.altitude;
    char AzStr[16], AltStr[16];
    fs_sexa(AzStr, parkAZ, 2, 3600);
    fs_sexa(AltStr, parkAlt, 2, 3600);
    LOGF_DEBUG("Setting current parking position to coordinates Az (%s) Alt (%s)...", AzStr, AltStr);
    SetAxis1Park(parkAZ);
    SetAxis2Park(parkAlt);
    return true;
}

bool LX200FS2::SetDefaultPark()
{
    // By default azimuth 0
    SetAxis1Park(0);

    // Altitude = latitude of observer
    SetAxis2Park(LocationNP[LOCATION_LATITUDE].getValue());

    return true;
}

bool LX200FS2::updateLocation(double latitude, double longitude, double elevation)
{
    INDI_UNUSED(latitude);
    INDI_UNUSED(longitude);
    INDI_UNUSED(elevation);
    return true;
}

bool LX200FS2::Goto(double r, double d)
{
    targetRA  = r;
    targetDEC = d;
    char RAStr[64], DecStr[64];

    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);


    if (!isSimulation())
    {
        if (setObjectRA(PortFD, targetRA, true) < 0 || (setObjectDEC(PortFD, targetDEC, true)) < 0)
        {
            EqNP.setState(IPS_ALERT);
            LOG_ERROR("Error setting RA/DEC.");
            EqNP.apply();
            return false;
        }

        if (Slew(PortFD))
        {
            EqNP.setState(IPS_ALERT);
            LOGF_ERROR("Error Slewing to JNow RA %s - DEC %s\n", RAStr, DecStr);
            EqNP.apply();
            slewError(1);
            return false;
        }
    }

    TrackState = SCOPE_SLEWING;
    EqNP.setState(IPS_BUSY);

    LOGF_INFO("Slewing to RA: %s - DEC: %s", RAStr, DecStr);
    return true;
}

bool LX200FS2::Sync(double ra, double dec)
{
    if (!isSimulation())
    {
        if (setObjectRA(PortFD, ra, true) < 0 || setObjectDEC(PortFD, dec, true) < 0)
        {
            EqNP.setState(IPS_ALERT);
            LOG_ERROR("Error setting RA/DEC. Unable to Sync.");
            EqNP.apply();
            return false;
        }

        char syncString[256];
        if (::Sync(PortFD, syncString) < 0)
        {
            EqNP.setState(IPS_ALERT);
            LOG_ERROR("Synchronization failed.");
            EqNP.apply();
            return false;
        }

    }

    currentRA  = ra;
    currentDEC = dec;
    LOG_INFO("Synchronization successful.");
    EqNP.setState(IPS_OK);
    NewRaDec(currentRA, currentDEC);
    return true;
}

