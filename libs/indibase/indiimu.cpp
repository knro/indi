/*
    IMU Driver Base Class
    Copyright (C) 2025 Jasem Mutlaq (mutlaqja@ikarustech.com)

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

#include "indiimu.h"
#include "connectionplugins/connectionserial.h"
#include "connectionplugins/connectioni2c.h"
#include "indicom.h"
#include "alignment/TelescopeDirectionVectorSupportFunctions.h"
#include "alignment/AlignmentSubsystemForDrivers.h"
#include <cmath>
#include <memory>

namespace INDI
{

IMU::IMU() : IMUInterface(this)
{
    // Constructor only initializes the interface base class.
    // Properties will be initialized in initProperties().
    serialConnection = nullptr;
    i2cConnection    = nullptr;
}

IMU::~IMU()
{
    delete serialConnection;
    delete i2cConnection;
}

void IMU::setSupportedConnections(const uint8_t &value)
{
    uint8_t mask = CONNECTION_SERIAL | CONNECTION_I2C | CONNECTION_NONE;

    if (value == 0 || (mask & value) == 0)
    {
        DEBUGF(Logger::DBG_ERROR, "Invalid connection mode %d", value);
        return;
    }

    imuConnection = value;
}

bool IMU::initProperties()
{
    DefaultDevice::initProperties();
    IMUInterface::initProperties(IMU_TAB);

    // Initialize driver-specific properties

    AstroCoordinatesNP[AXIS1].fill("AXIS1", "Axis 1 (deg)", "%.2f", 0, 360, 0, 0);
    AstroCoordinatesNP[AXIS2].fill("AXIS2", "Axis 2 (deg)", "%.2f", 0, 360, 0, 0);
    AstroCoordinatesNP.fill(getDeviceName(), "COORDINATES", "Coordinates", COORDINATES_TAB.c_str(), IP_RO,
                            0, IPS_IDLE);

    AstroCoordsTypeSP[COORD_EQUATORIAL].fill("EQUATORIAL", "Equatorial (HA/DEC)", ISS_ON);
    AstroCoordsTypeSP[COORD_ALTAZ].fill("ALTAZ", "Alt-Az (AZ/ALT)", ISS_OFF);
    AstroCoordsTypeSP.fill(getDeviceName(), "COORDS_TYPE", "Coordinate Type", COORDINATES_TAB.c_str(), IP_RW,
                           ISR_1OFMANY, 0, IPS_IDLE);
    AstroCoordsTypeSP.load();

    IMUFrameSP[ENU].fill("ENU", "East-North-Up", ISS_ON);
    IMUFrameSP[NWU].fill("NWU", "North-West-Up", ISS_OFF);
    IMUFrameSP[SWU].fill("SWU", "South-West-Up", ISS_OFF);
    IMUFrameSP.fill(getDeviceName(), "IMU_FRAME", "IMU Frame", IMU_TAB.c_str(), IP_RW, ISR_1OFMANY, 0, IPS_IDLE);
    IMUFrameSP.load();

    OrientationAdjustmentsNP[ROLL_MULTIPLIER].fill("ROLL_M", "Roll Multiplier", "%.2f", -1.0, 1.0, 0.1, 1.0);
    OrientationAdjustmentsNP[PITCH_MULTIPLIER].fill("PITCH_M", "Pitch Multiplier", "%.2f", -1.0, 1.0, 0.1, 1.0);
    OrientationAdjustmentsNP[YAW_MULTIPLIER].fill("YAW_M", "Yaw Multiplier", "%.2f", -1.0, 1.0, 0.1, 1.0);
    OrientationAdjustmentsNP[ROLL_OFFSET].fill("ROLL_O", "Roll Offset (deg)", "%.2f", -360.0, 360.0, 10, 0.0);
    OrientationAdjustmentsNP[PITCH_OFFSET].fill("PITCH_O", "Pitch Offset (deg)", "%.2f", -360.0, 360.0, 10, 0.0);
    OrientationAdjustmentsNP[YAW_OFFSET].fill("YAW_O", "Yaw Offset (deg)", "%.2f", -360.0, 360.0, 10, 0.0);
    OrientationAdjustmentsNP.fill(getDeviceName(), "ORIENTATION_ADJUSTMENTS", "Orientation Adjustments", IMU_TAB.c_str(), IP_RW,
                                  0, IPS_IDLE);
    OrientationAdjustmentsNP.load();

    SyncAxisNP[AXIS1].fill("SYNC_AXIS1", "Sync Axis 1 (deg)", "%.2f", -360, 360, 10, 0);
    SyncAxisNP[AXIS2].fill("SYNC_AXIS2", "Sync Axis 2 (deg)", "%.2f", -360, 360, 10, 0);
    SyncAxisNP.fill(getDeviceName(), "SYNC_AXIS", "Sync Axis", COORDINATES_TAB.c_str(), IP_RW, 0, IPS_IDLE);

    TelescopeVectorNP[TELESCOPE_VECTOR_X].fill("TELESCOPE_VECTOR_X", "Telescope Vector X", "%.2f", -1.0, 1.0, 0.1, 1.0);
    TelescopeVectorNP[TELESCOPE_VECTOR_Y].fill("TELESCOPE_VECTOR_Y", "Telescope Vector Y", "%.2f", -1.0, 1.0, 0.1, 0.0);
    TelescopeVectorNP[TELESCOPE_VECTOR_Z].fill("TELESCOPE_VECTOR_Z", "Telescope Vector Z", "%.2f", -1.0, 1.0, 0.1, 0.0);
    TelescopeVectorNP.fill(getDeviceName(), "TELESCOPE_VECTOR", "Telescope Vector", IMU_TAB.c_str(), IP_RW, 0, IPS_IDLE);
    TelescopeVectorNP.load();

    GeographicCoordNP[LOCATION_LATITUDE].fill("LAT", "Lat (dd:mm:ss.s)", "%012.8m", -90, 90, 0, 0.0);
    GeographicCoordNP[LOCATION_LONGITUDE].fill("LONG", "Lon (dd:mm:ss.s)", "%012.8m", 0, 360, 0, 0.0);
    GeographicCoordNP[LOCATION_ELEVATION].fill("ELEV", "Elevation (m)", "%g", -200, 10000, 0, 0);
    GeographicCoordNP.fill(getDeviceName(), "GEOGRAPHIC_COORD", "Location", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);
    GeographicCoordNP.load();

    MagneticDeclinationNP[0].fill("MAGNETIC_DECLINATION", "Magnetic Declination", "%.4f", -180, 180, 0, 0);
    MagneticDeclinationNP.fill(getDeviceName(), "MAGNETIC_DECLINATION", "Magnetic Declination", MAIN_CONTROL_TAB, IP_RW, 0,
                               IPS_IDLE);
    MagneticDeclinationNP.load();

    if (imuConnection & CONNECTION_SERIAL)
    {
        serialConnection = new Connection::Serial(this);
        serialConnection->registerHandshake([&]()
        {
            return callHandshake();
        });
        registerConnection(serialConnection);
    }

    if (imuConnection & CONNECTION_I2C)
    {
        i2cConnection = new Connection::I2C(this);
        i2cConnection->setDefaultBusPath("/dev/i2c-1");
        i2cConnection->registerHandshake([&]()
        {
            return callHandshake();
        });
        registerConnection(i2cConnection);
    }

    return true;
}

bool IMU::updateProperties()
{
    DefaultDevice::updateProperties();
    IMUInterface::updateProperties();

    if (isConnected())
    {
        // Define driver-specific properties when connected
        defineProperty(AstroCoordinatesNP);
        defineProperty(AstroCoordsTypeSP);
        defineProperty(IMUFrameSP);
        defineProperty(OrientationAdjustmentsNP);
        defineProperty(SyncAxisNP);
        defineProperty(TelescopeVectorNP);
        defineProperty(GeographicCoordNP);
        defineProperty(MagneticDeclinationNP);
    }
    else
    {
        // Delete driver-specific properties when disconnected
        deleteProperty(AstroCoordinatesNP);
        deleteProperty(AstroCoordsTypeSP);
        deleteProperty(IMUFrameSP);
        deleteProperty(OrientationAdjustmentsNP);
        deleteProperty(SyncAxisNP);
        deleteProperty(TelescopeVectorNP);
        deleteProperty(GeographicCoordNP);
        deleteProperty(MagneticDeclinationNP);
    }
    return true;
}

bool IMU::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (IMUInterface::processNumber(dev, name, values, names, n))
        return true;

    if (OrientationAdjustmentsNP.isNameMatch(name))
    {
        updateProperty(OrientationAdjustmentsNP, values, names, n, [this]()
        {
            // When orientation adjustments change, recalculate astro coordinates
            // and also update the IMU's internal orientation properties.
            // This will trigger SetOrientationData to apply the new adjustments.
            SetOrientationData(last_q_i, last_q_j, last_q_k, last_q_w);
            return true;
        }, true);
        return true;
    }

    if (TelescopeVectorNP.isNameMatch(name))
    {
        updateProperty(TelescopeVectorNP, values, names, n, [this]()
        {
            RecalculateAstroCoordinates();
            return true;
        }, true);
        return true;
    }

    if (GeographicCoordNP.isNameMatch(name))
    {
        updateProperty(GeographicCoordNP, values, names, n, [this]()
        {
            RecalculateAstroCoordinates();
            return true;
        }, true);
        return true;
    }

    if (MagneticDeclinationNP.isNameMatch(name))
    {
        updateProperty(MagneticDeclinationNP, values, names, n, [&]()
        {
            RecalculateAstroCoordinates();
            return true;
        }, true);
        return true;
    }

    if (SyncAxisNP.isNameMatch(name))
    {
        updateProperty(SyncAxisNP, values, names, n, [this, values]()
        {
            INDI::AlignmentSubsystem::TelescopeDirectionVectorSupportFunctions tdvFunctions;

            INDI::AlignmentSubsystem::TelescopeDirectionVector currentVector, syncVector;

            if (AstroCoordsTypeSP[COORD_EQUATORIAL].s == ISS_ON)
            {
                INDI::IEquatorialCoordinates currentEq, syncEq;
                currentEq.rightascension = AstroCoordinatesNP[AXIS1].getValue() / 15.0; // Convert degrees to hours
                currentEq.declination    = AstroCoordinatesNP[AXIS2].getValue();
                currentVector            = tdvFunctions.TelescopeDirectionVectorFromLocalHourAngleDeclination(currentEq);

                syncEq.rightascension = values[AXIS1] / 15.0; // Convert degrees to hours
                syncEq.declination    = values[AXIS2];
                syncVector            = tdvFunctions.TelescopeDirectionVectorFromLocalHourAngleDeclination(syncEq);
            }
            else
            {
                INDI::IHorizontalCoordinates currentHor, syncHor;
                currentHor.azimuth  = AstroCoordinatesNP[AXIS1].getValue();
                currentHor.altitude = AstroCoordinatesNP[AXIS2].getValue();
                currentVector       = tdvFunctions.TelescopeDirectionVectorFromAltitudeAzimuth(currentHor);

                syncHor.azimuth  = values[AXIS1];
                syncHor.altitude = values[AXIS2];
                syncVector       = tdvFunctions.TelescopeDirectionVectorFromAltitudeAzimuth(syncHor);
            }

            // Calculate the rotation axis and angle
            INDI::AlignmentSubsystem::TelescopeDirectionVector rotationAxis = currentVector * syncVector;
            rotationAxis.Normalise();
            double rotationAngle = acos(currentVector ^ syncVector);

            // Convert axis-angle to quaternion
            double s = sin(rotationAngle / 2.0);
            double w = cos(rotationAngle / 2.0);
            double i = rotationAxis.x * s;
            double j = rotationAxis.y * s;
            double k = rotationAxis.z * s;

            // Convert the rotation quaternion to Euler angles (roll, pitch, yaw)
            double roll, pitch, yaw;
            QuaternionToEuler(i, j, k, w, roll, pitch, yaw);

            // Apply the calculated offsets to the orientation adjustments
            OrientationAdjustmentsNP[ROLL_OFFSET].setValue(OrientationAdjustmentsNP[ROLL_OFFSET].getValue() + RAD_TO_DEG(roll));
            OrientationAdjustmentsNP[PITCH_OFFSET].setValue(OrientationAdjustmentsNP[PITCH_OFFSET].getValue() + RAD_TO_DEG(pitch));
            OrientationAdjustmentsNP[YAW_OFFSET].setValue(OrientationAdjustmentsNP[YAW_OFFSET].getValue() + RAD_TO_DEG(yaw));

            // Recalculate the astro coordinates with the new adjustments
            RecalculateAstroCoordinates();
            return true;
        });
        return true;
    }

    return DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool IMU::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (IMUInterface::processSwitch(dev, name, states, names, n))
        return true;

    if (AstroCoordsTypeSP.isNameMatch(name))
    {
        updateProperty(AstroCoordsTypeSP, states, names, n, [this]()
        {
            RecalculateAstroCoordinates();
            return true;
        }, true);
        return true;
    }

    if (IMUFrameSP.isNameMatch(name))
    {
        updateProperty(IMUFrameSP, states, names, n, [this]()
        {
            RecalculateAstroCoordinates();
            return true;
        }, true);
        return true;
    }


    return DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool IMU::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (IMUInterface::processText(dev, name, texts, names, n))
        return true;

    return DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool IMU::saveConfigItems(FILE *fp)
{
    IMUInterface::saveConfigItems(fp);
    DefaultDevice::saveConfigItems(fp);

    // Save driver-specific properties
    AstroCoordsTypeSP.save(fp);
    IMUFrameSP.save(fp);
    OrientationAdjustmentsNP.save(fp);
    TelescopeVectorNP.save(fp);
    GeographicCoordNP.save(fp);

    return true;
}

bool IMU::Handshake()
{
    // Default implementation does nothing and returns false
    // Concrete drivers should override this method
    return false;
}

bool IMU::callHandshake()
{
    if (imuConnection > 0)
    {
        if (getActiveConnection() == serialConnection)
            PortFD = serialConnection->getPortFD();
        else if (getActiveConnection() == i2cConnection)
            PortFD = i2cConnection->getPortFD();
    }

    return Handshake();
}

// Helper function to convert quaternion to Euler angles (roll, pitch, yaw)
// Angles are in radians
void IMU::QuaternionToEuler(double i, double j, double k, double w, double &roll, double &pitch, double &yaw)
{
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (w * i + j * k);
    double cosr_cosp = 1 - 2 * (i * i + j * j);
    roll             = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (w * j - k * i);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * k + i * j);
    double cosy_cosp = 1 - 2 * (j * j + k * k);
    yaw              = std::atan2(siny_cosp, cosy_cosp);
}

// Helper function to convert Euler angles (roll, pitch, yaw) in radians to a quaternion
void IMU::EulerToQuaternion(double roll, double pitch, double yaw, double &i, double &j, double &k, double &w)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    w = cr * cp * cy + sr * sp * sy;
    i = sr * cp * cy - cr * sp * sy;
    j = cr * sp * cy + sr * cp * sy;
    k = cr * cp * sy - sr * sp * cy;
}

void IMU::RecalculateAstroCoordinates()
{
    // Log raw quaternion values
    DEBUGF(Logger::DBG_DEBUG, "IMU: Recalculating Astro Coordinates from stored Quaternion: i=%.4f, j=%.4f, k=%.4f, w=%.4f",
           last_q_i, last_q_j, last_q_k, last_q_w);

    double rollRad, pitchRad, yawRad;
    QuaternionToEuler(last_q_i, last_q_j, last_q_k, last_q_w, rollRad, pitchRad, yawRad);

    // Convert radians to degrees for INDI properties
    // The quaternion (last_q_w, last_q_i, last_q_j, last_q_k) now represents the
    // IMU's orientation after applying user-defined multipliers and offsets.
    double qw = last_q_w;
    double qx = last_q_i;
    double qy = last_q_j;
    double qz = last_q_k;

    // Step 1: Define the telescope's pointing vector in the IMU's own reference frame.
    // This vector is now configurable via the TELESCOPE_VECTOR property.
    double vx = TelescopeVectorNP[TELESCOPE_VECTOR_X].getValue();
    double vy = TelescopeVectorNP[TELESCOPE_VECTOR_Y].getValue();
    double vz = TelescopeVectorNP[TELESCOPE_VECTOR_Z].getValue();

    // Step 2: Rotate this vector by the IMU's adjusted quaternion to get the pointing
    // direction in the local horizon frame. This is a standard quaternion-vector rotation.
    // Formula: V' = q * V * q_conjugate, where V is a pure quaternion (0, vx, vy, vz)
    // Expanded formula for rotated vector components (x', y', z'):
    double x_hor = vx * (1 - 2 * qy * qy - 2 * qz * qz) + vy * (2 * qx * qy - 2 * qz * qw) + vz * (2 * qx * qz + 2 * qy * qw);
    double y_hor = vx * (2 * qx * qy + 2 * qz * qw) + vy * (1 - 2 * qx * qx - 2 * qz * qz) + vz * (2 * qy * qz - 2 * qx * qw);
    double z_hor = vx * (2 * qx * qz - 2 * qy * qw) + vy * (2 * qy * qz + 2 * qx * qw) + vz * (1 - 2 * qx * qx - 2 * qy * qy);

    INDI::AlignmentSubsystem::TelescopeDirectionVector imu_vector(x_hor, y_hor, z_hor);
    INDI::AlignmentSubsystem::TelescopeDirectionVector horizontal_vector;

    // The imu_vector is in the IMU's native coordinate system. We need to convert
    // it to the standard ENU (East-North-Up) frame for the rest of the calculations.
    switch (IMUFrameSP.findOnSwitchIndex())
    {
        case ENU:
            // No conversion needed
            horizontal_vector = imu_vector;
            break;
        case NWU:
            // North-West-Up to East-North-Up
            // X_enu = -Y_nwu
            // Y_enu =  X_nwu
            // Z_enu =  Z_nwu
            horizontal_vector.x = -imu_vector.y;
            horizontal_vector.y = imu_vector.x;
            horizontal_vector.z = imu_vector.z;
            break;
        case SWU:
            // South-West-Up to East-North-Up
            // X_enu = -Y_swu
            // Y_enu = -X_swu
            // Z_enu =  Z_swu
            horizontal_vector.x = -imu_vector.y;
            horizontal_vector.y = -imu_vector.x;
            horizontal_vector.z = imu_vector.z;
            break;
    }

    // Log the horizontal vector
    DEBUGF(Logger::DBG_DEBUG, "IMU: Horizontal Vector: X=%.4f, Y=%.4f, Z=%.4f", horizontal_vector.x, horizontal_vector.y,
           horizontal_vector.z);

    if (AstroCoordsTypeSP[COORD_EQUATORIAL].s == ISS_ON)
    {
        // Step 3: Rotate the horizontal vector to the equatorial frame based on latitude.
        double latitude = GeographicCoordNP[LOCATION_LATITUDE].getValue();
        double lat_rad = DEG_TO_RAD(latitude);
        double sin_lat = sin(lat_rad);
        double cos_lat = cos(lat_rad);

        // The transformation from the horizontal frame (East, North, Up) to the
        // equatorial frame (South, West, Celestial Pole).
        // horizontal_vector.x = East component (v_e)
        // horizontal_vector.y = North component (v_n)
        // horizontal_vector.z = Up component (v_u)
        //
        // The equatorial frame is defined as:
        // X_eq points to the meridian, Y_eq points West, Z_eq points to the North Celestial Pole
        //
        // The correct transformation is:
        // x_eq =  v_u * cos(lat) - v_n * sin(lat)
        // y_eq = -v_e
        // z_eq =  v_u * sin(lat) + v_n * cos(lat)
        double x_eq = (horizontal_vector.z * cos_lat) - (horizontal_vector.y * sin_lat);
        double y_eq = -horizontal_vector.x;
        double z_eq = (horizontal_vector.z * sin_lat) + (horizontal_vector.y * cos_lat);

        INDI::AlignmentSubsystem::TelescopeDirectionVector equatorial_vector(x_eq, y_eq, z_eq);

        // Log the equatorial vector
        DEBUGF(Logger::DBG_DEBUG, "IMU: Equatorial Vector: X=%.4f, Y=%.4f, Z=%.4f", x_eq, y_eq, z_eq);

        // Step 4: Extract HA and Dec from the final equatorial vector.
        INDI::IEquatorialCoordinates eq_coords;
        INDI::AlignmentSubsystem::TelescopeDirectionVectorSupportFunctions tdv_functions;
        tdv_functions.LocalHourAngleDeclinationFromTelescopeDirectionVector(equatorial_vector, eq_coords);

        // Step 5: Update INDI properties.
        // The support function returns HA in hours, so we convert it to degrees for the property.
        AstroCoordinatesNP[AXIS1].setValue(eq_coords.rightascension * 15.0); // HA in degrees
        AstroCoordinatesNP[AXIS2].setValue(eq_coords.declination);
        DEBUGF(Logger::DBG_DEBUG, "IMU: Calculated HA=%.2f deg, Dec=%.2f deg", eq_coords.rightascension * 15.0,
               eq_coords.declination);
    }
    else // Alt-Az calculation
    {
        // For Alt-Az, we must convert the ENU horizontal_vector to the NWU
        // (North, West, Up) frame expected by the support function.
        INDI::AlignmentSubsystem::TelescopeDirectionVector nwu_vector(horizontal_vector.y, -horizontal_vector.x,
                horizontal_vector.z);

        // For Alt-Az, we just need to convert the horizontal_vector to spherical coordinates.
        INDI::IHorizontalCoordinates horiz_coords;
        INDI::AlignmentSubsystem::TelescopeDirectionVectorSupportFunctions tdv_functions;
        tdv_functions.AltitudeAzimuthFromTelescopeDirectionVector(nwu_vector, horiz_coords);

        AstroCoordinatesNP[AXIS1].setValue(horiz_coords.azimuth);
        AstroCoordinatesNP[AXIS2].setValue(horiz_coords.altitude);
        DEBUGF(Logger::DBG_DEBUG, "IMU: Calculated Az=%.2f deg, Alt=%.2f deg", horiz_coords.azimuth, horiz_coords.altitude);
    }

    // Common code to send the update
    AstroCoordinatesNP.setState(IPS_OK);
    AstroCoordinatesNP.apply();
}

// Implement virtual functions from IMUInterface
bool IMU::SetOrientationData(double i, double j, double k, double w)
{
    // Log raw quaternion values
    DEBUGF(Logger::DBG_DEBUG, "IMU: Raw Quaternion: i=%.4f, j=%.4f, k=%.4f, w=%.4f", i, j, k, w);

    double rawRollRad, rawPitchRad, rawYawRad;
    QuaternionToEuler(i, j, k, w, rawRollRad, rawPitchRad, rawYawRad);

    // Get multipliers and offsets from properties
    double rollMultiplier  = OrientationAdjustmentsNP[ROLL_MULTIPLIER].getValue();
    double pitchMultiplier = OrientationAdjustmentsNP[PITCH_MULTIPLIER].getValue();
    double yawMultiplier   = OrientationAdjustmentsNP[YAW_MULTIPLIER].getValue();
    double rollOffset      = OrientationAdjustmentsNP[ROLL_OFFSET].getValue() * M_PI / 180.0; // Convert offset to radians
    double pitchOffset     = OrientationAdjustmentsNP[PITCH_OFFSET].getValue() * M_PI / 180.0;
    double yawOffset       = OrientationAdjustmentsNP[YAW_OFFSET].getValue() * M_PI / 180.0;
    double magneticDeclinationRad = MagneticDeclinationNP[0].getValue() * M_PI / 180.0; // Convert to radians

    // Apply adjustments
    double adjustedRollRad  = rawRollRad * rollMultiplier + rollOffset;
    double adjustedPitchRad = rawPitchRad * pitchMultiplier + pitchOffset;
    double adjustedYawRad   = rawYawRad * yawMultiplier + yawOffset + magneticDeclinationRad;

    // Convert adjusted radians to degrees for INDI properties
    double adjustedRollDeg  = adjustedRollRad * 180.0 / M_PI;
    double adjustedPitchDeg = adjustedPitchRad * 180.0 / M_PI;
    double adjustedYawDeg   = adjustedYawRad * 180.0 / M_PI;

    // Log adjusted Euler angles
    DEBUGF(Logger::DBG_DEBUG, "IMU: Adjusted Euler Angles (deg): Roll=%.2f, Pitch=%.2f, Yaw=%.2f", adjustedRollDeg,
           adjustedPitchDeg, adjustedYawDeg);

    // Update INDI Orientation properties (Roll, Pitch, Yaw in degrees)
    OrientationNP[ORIENTATION_ROLL].setValue(rawRollRad * 180.0 / M_PI);
    OrientationNP[ORIENTATION_PITCH].setValue(rawPitchRad * 180.0 / M_PI);
    OrientationNP[ORIENTATION_YAW].setValue(rawYawRad * 180.0 / M_PI);
    OrientationNP[ORIENTATION_QUATERNION_W].setValue(w);
    OrientationNP.setState(IPS_OK);
    OrientationNP.apply();

    // Store adjusted quaternion values for recalculation
    // Convert adjusted Euler angles back to a quaternion for storage
    EulerToQuaternion(adjustedRollRad, adjustedPitchRad, adjustedYawRad, last_q_i, last_q_j, last_q_k, last_q_w);

    // Recalculate astronomical coordinates
    RecalculateAstroCoordinates();

    return true;
}

bool IMU::SetAccelerationData(double x, double y, double z)
{
    AccelerationNP[ACCELERATION_X].setValue(x);
    AccelerationNP[ACCELERATION_Y].setValue(y);
    AccelerationNP[ACCELERATION_Z].setValue(z);
    AccelerationNP.setState(IPS_OK);
    AccelerationNP.apply();
    return true;
}

bool IMU::SetGyroscopeData(double x, double y, double z)
{
    GyroscopeNP[GYROSCOPE_X].setValue(x);
    GyroscopeNP[GYROSCOPE_Y].setValue(y);
    GyroscopeNP[GYROSCOPE_Z].setValue(z);
    GyroscopeNP.setState(IPS_OK);
    GyroscopeNP.apply();
    return true;
}

bool IMU::SetMagnetometerData(double x, double y, double z)
{
    MagnetometerNP[MAGNETOMETER_X].setValue(x);
    MagnetometerNP[MAGNETOMETER_Y].setValue(y);
    MagnetometerNP[MAGNETOMETER_Z].setValue(z);
    MagnetometerNP.setState(IPS_OK);
    MagnetometerNP.apply();
    return true;
}
bool IMU::SetCalibrationStatus(int sys, int gyro, int accel, int mag)
{
    INDI_UNUSED(sys);
    INDI_UNUSED(gyro);
    INDI_UNUSED(accel);
    INDI_UNUSED(mag);
    return false;
}
bool IMU::StartCalibration()
{
    return false;
}
bool IMU::SaveCalibrationData()
{
    return false;
}
bool IMU::LoadCalibrationData()
{
    return false;
}
bool IMU::ResetCalibration()
{
    return false;
}
bool IMU::SetPowerMode(const std::string &mode)
{
    INDI_UNUSED(mode);
    return false;
}
bool IMU::SetOperationMode(const std::string &mode)
{
    INDI_UNUSED(mode);
    return false;
}
bool IMU::SetDistanceUnits(bool metric)
{
    INDI_UNUSED(metric);
    return false;
}
bool IMU::SetAngularUnits(bool degrees)
{
    INDI_UNUSED(degrees);
    return false;
}
bool IMU::SetUpdateRate(double rate)
{
    INDI_UNUSED(rate);
    return false;
}
bool IMU::SetDeviceInfo(const std::string &chipID, const std::string &firmwareVersion, const std::string &sensorStatus)
{
    INDI_UNUSED(chipID);
    INDI_UNUSED(firmwareVersion);
    INDI_UNUSED(sensorStatus);
    return false;
}
bool IMU::SetTemperature(double temperature)
{
    INDI_UNUSED(temperature);
    return false;
}
bool IMU::SetStabilityMonitoring(double vibrationLevel, double stabilityThreshold)
{
    INDI_UNUSED(vibrationLevel);
    INDI_UNUSED(stabilityThreshold);
    return false;
}

}
