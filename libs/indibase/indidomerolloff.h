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

#pragma once

#include "indidome.h"

namespace INDI
{

/**
 * \class RollOffDome
 * \brief Intermediate base class for linear roll-off roof enclosures.
 *
 * Implements the shared roll-off state machine: opening the roof maps to DOME_CW,
 * closing maps to DOME_CCW, and Park()/UnPark() are aliases for close/open.
 * Concrete drivers only need to implement the hardware primitives.
 */
class RollOffDome : public Dome
{
    public:
        RollOffDome();
        virtual ~RollOffDome() = default;

    protected:
        virtual bool initProperties() override;
        virtual void TimerHit() override;

        virtual IPState Move(DomeDirection dir, DomeMotionCommand operation) override;
        virtual IPState Park() override;
        virtual IPState UnPark() override;
        virtual bool Abort() override;

        /////////////////////////////////////////////////////////////////////
        /// Hardware interface (implemented by the concrete driver)
        /////////////////////////////////////////////////////////////////////
        /** \brief Command the roof to open. Return true if the command was accepted. */
        virtual bool startOpenRoof() = 0;
        /** \brief Command the roof to close. Return true if the command was accepted. */
        virtual bool startCloseRoof() = 0;
        /** \brief Stop any roof motion. Return true if the command was accepted. */
        virtual bool stopRoof() = 0;
        /** \brief Return true if the fully-open limit switch is asserted. */
        virtual bool isFullyOpen() = 0;
        /** \brief Return true if the fully-closed limit switch is asserted. */
        virtual bool isFullyClosed() = 0;

        /** \brief Optional hook called at the start of every TimerHit() poll. */
        virtual void updateRoof() {}

        /** \brief Roof motion status. */
        enum RoofStatus
        {
            ROOF_IDLE,
            ROOF_OPENING,
            ROOF_CLOSING
        };

        RoofStatus getRoofStatus() const
        {
            return m_RoofStatus;
        }

    private:
        RoofStatus m_RoofStatus { ROOF_IDLE };
        bool m_RoofAborted { false };
};

}
