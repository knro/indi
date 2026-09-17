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

#pragma once

#include "indidomerolloff.h"

class RollOff : public INDI::RollOffDome
{
    public:
        RollOff();
        virtual ~RollOff() = default;

        virtual const char *getDefaultName() override;
        virtual bool initProperties() override;
        virtual bool saveConfigItems(FILE *fp) override;

    protected:
        bool Connect() override;
        bool Disconnect() override;

        // INDI::RollOffDome hardware interface
        bool startOpenRoof() override;
        bool startCloseRoof() override;
        bool stopRoof() override;
        bool isFullyOpen() override;
        bool isFullyClosed() override;
        void updateRoof() override;

    private:
        float CalcTimeLeft(timeval start);

        ISState fullOpenLimitSwitch { ISS_ON };
        ISState fullClosedLimitSwitch { ISS_OFF };
        double MotionRequest { 0 };
        struct timeval MotionStart
        {
            0, 0
        };
};
