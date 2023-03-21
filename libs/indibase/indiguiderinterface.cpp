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

#include "indiguiderinterface.h"

#include <cstring>

namespace INDI
{

///////////////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////////////
GuiderInterface::GuiderInterface(DefaultDevice *device,
                                 std::function<IPState(INDI_DIR_NS, uint32_t)> &ns,
                                 std::function<IPState(INDI_DIR_WE, uint32_t)> &we) : AbstractInterface(device),
    m_GuideNSFP(ns),
    m_GuideWEFP(we)
{
}

///////////////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////////////
void GuiderInterface::initProperties(const char *group)
{
    GuideNSNP[DIRECTION_NORTH].fill("TIMED_GUIDE_N", "North (ms)", "%.f", 0, 60000, 100, 0);
    GuideNSNP[DIRECTION_SOUTH].fill("TIMED_GUIDE_S", "South (ms)", "%.f", 0, 60000, 100, 0);
    GuideNSNP.fill(getDeviceName(), "TELESCOPE_TIMED_GUIDE_NS", "Guide N/S", group, IP_RW, 60, IPS_IDLE);

    GuideWENP[DIRECTION_WEST].fill("TIMED_GUIDE_W", "West (ms)", "%.f", 0, 60000, 100, 0);
    GuideWENP[DIRECTION_EAST].fill("TIMED_GUIDE_E", "East (ms)", "%.f", 0, 60000, 100, 0);
    GuideWENP.fill(getDeviceName(), "TELESCOPE_TIMED_GUIDE_WE", "Guide E/W", group, IP_RW, 60, IPS_IDLE);
}

///////////////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////////////
bool GuiderInterface::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    INDI_UNUSED(dev);

    // North/South
    if (GuideNSNP.isNameMatch(name))
    {
        GuideNSNP.update(values, names, n);

        if (GuideNSNP[DIRECTION_NORTH].getValue() != 0)
        {
            GuideNSNP[DIRECTION_SOUTH].setValue(0);
            GuideNSNP.setState(m_GuideNSFP(DIRECTION_NORTH, values[0]));
        }
        else if (GuideNSNP[DIRECTION_SOUTH].getValue() != 0)
            GuideNSNP.setState(m_GuideNSFP(DIRECTION_SOUTH, values[0]));

        GuideNSNP.apply();
        return true;
    }

    // West/East
    if (GuideWENP.isNameMatch(name))
    {
        GuideWENP.update(values, names, n);

        if (GuideWENP[DIRECTION_WEST].getValue() != 0)
        {
            GuideWENP[DIRECTION_EAST].setValue(0);
            GuideWENP.setState(m_GuideWEFP(DIRECTION_WEST, values[0]));
        }
        else if (GuideWENP[DIRECTION_EAST].getValue() != 0)
            GuideWENP.setState(m_GuideWEFP(DIRECTION_EAST, values[0]));

        GuideWENP.apply();
        return true;
    }

    return false;
}

///////////////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////////////
void GuiderInterface::setGuideComplete(INDI_EQ_AXIS axis)
{
    switch (axis)
    {
        case AXIS_DE:
            GuideNSNP.setState(IPS_IDLE);
            GuideNSNP.apply(nullptr);
            break;

        case AXIS_RA:
            GuideWENP.setState(IPS_IDLE);
            GuideWENP.apply();
            break;
    }
}

}
