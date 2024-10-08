/*******************************************************************************
  Copyright(c) 2021 Chrysikos Efstathios. All rights reserved.

  Pegasus FlatMaster

  2022-06-07 Jasem Mutlaq: Use lightbox interface properly.

  This program is free software; you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; see the file COPYING.LIB.  If not, write to
  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
  Boston, MA 02110-1301, USA.

  The full GNU General Public License is included in this distribution in the
  file called LICENSE.
*******************************************************************************/

#include "pegasus_flatmaster.h"

#include "indicom.h"
#include "connectionplugins/connectionserial.h"

#include <cerrno>
#include <cstring>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <math.h>


static std::unique_ptr<PegasusFlatMaster> flatmaster(new PegasusFlatMaster());


PegasusFlatMaster::PegasusFlatMaster() : LightBoxInterface(this)
{
    setVersion(1, 2);
}

bool PegasusFlatMaster::initProperties()
{
    INDI::DefaultDevice::initProperties();

    // Firmware version
    IUFillText(&FirmwareT[0], "Version", "Version", nullptr);
    IUFillTextVector(&FirmwareTP, FirmwareT, 1, getDeviceName(), "Firmware", "Firmware", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    LI::initProperties(MAIN_CONTROL_TAB, CAN_DIM);

    setDriverInterface(AUX_INTERFACE | LIGHTBOX_INTERFACE);

    LightIntensityNP[0].setMin(0);
    LightIntensityNP[0].setMax(100);
    LightIntensityNP[0].setStep(1);

    addAuxControls();

    serialConnection = new Connection::Serial(this);
    serialConnection->setDefaultBaudRate(Connection::Serial::B_9600);
    serialConnection->registerHandshake([&]()
    {
        return Ack();
    });


    registerConnection(serialConnection);
    return true;
}

void PegasusFlatMaster::ISGetProperties(const char *dev)
{
    INDI::DefaultDevice::ISGetProperties(dev);

    // Get Light box properties
    LI::ISGetProperties(dev);
}

bool PegasusFlatMaster::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    if (isConnected())
    {
        defineProperty(&FirmwareTP);
    }
    else
    {
        deleteProperty(FirmwareTP.name);
    }

    LI::updateProperties();
    return true;
}

const char *PegasusFlatMaster::getDefaultName()
{
    return "Pegasus FlatMaster";
}

void PegasusFlatMaster::updateFirmwareVersion()
{
    char response[16] = {0};

    if(sendCommand("V", response))
    {
        IUSaveText(&FirmwareT[0], response);
        FirmwareTP.s = IPS_OK;
        IDSetText(&FirmwareTP, nullptr);
    }
    else
    {
        FirmwareTP.s = IPS_ALERT;
        LOG_ERROR("Error on updateFirmware.");
    }
}

bool PegasusFlatMaster::Ack()
{
    PortFD = serialConnection->getPortFD();

    char response[16] = {0};
    if(sendCommand("#", response))
    {
        if(strstr("OK_FM", response) != nullptr)
        {
            updateFirmwareVersion();
            return  true;
        }
    }
    else
    {
        LOG_ERROR("Ack failed.");
        return false;
    }

    return false;
}

bool PegasusFlatMaster::EnableLightBox(bool enable)
{
    char response[16] = {0};
    char cmd[16] = {0};

    snprintf(cmd, 16, "E:%d", enable ? 1 : 0);

    if(sendCommand(cmd, response))
    {
        if(strstr(cmd, response) != nullptr)
        {
            return  true;
        }
    }
    else
    {
        LOGF_ERROR("Error on EnableLightBox. %s", response);
        return false;
    }
    return false;
}

bool PegasusFlatMaster::SetLightBoxBrightness(uint16_t value)
{
    if(LightSP[FLAT_LIGHT_ON].getState() != ISS_ON)
    {
        LOG_ERROR("You must set On the Flat Light first.");
        return false;
    }

    char response[16] = {0};
    char cmd[16] = {0};

    uint16_t result = floor((100 - value - 0) * (255 - 20) / (100 - 0) + 20);

    snprintf(cmd, 16, "L:%d", result);

    if(sendCommand(cmd, response))
    {
        if(strstr(cmd, response) != nullptr)
        {
            return  true;
        }
    }
    else
    {
        LOGF_ERROR("Error on SetLightBoxBrightness. %s", response);
        return false;
    }
    return false;
}

bool PegasusFlatMaster::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (LI::processNumber(dev, name, values, names, n))
        return true;

    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool PegasusFlatMaster::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (LI::processText(dev, name, texts, names, n))
            return true;
    }

    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool PegasusFlatMaster::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (LI::processSwitch(dev, name, states, names, n))
            return true;
    }

    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool PegasusFlatMaster::ISSnoopDevice(XMLEle *root)
{
    LI::snoop(root);

    return INDI::DefaultDevice::ISSnoopDevice(root);
}

bool PegasusFlatMaster::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);

    return LI::saveConfigItems(fp);
}

bool PegasusFlatMaster::sendCommand(const char *command, char *res)
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;
    char errstr[MAXRBUF];

    char cmd[7] = {0};
    snprintf(cmd, 7, "%s\n", command);

    LOGF_DEBUG("CMD <%#02X>", cmd[0]);

    tcflush(PortFD, TCIOFLUSH);

    if ((rc = tty_write(PortFD, cmd, strlen(cmd), &nbytes_written)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("command: %s error: %s.", cmd, errstr);
        return false;
    }

    if ((rc = tty_read_section(PortFD, res, 0xA, 3, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("command: %s error: %s.", cmd, errstr);
        return  false;
    }

    // Get rid of 0xA
    res[nbytes_read - 1] = 0;

    if(res[nbytes_read - 2] == '\r')
        res[nbytes_read - 2] = 0;

    LOGF_DEBUG("RES <%s>", res);

    tcflush(PortFD, TCIOFLUSH);

    return true;
}
