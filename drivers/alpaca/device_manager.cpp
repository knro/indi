/*******************************************************************************
  Copyright(c) 2025 Jasem Mutlaq. All rights reserved.

  INDI Alpaca Device Manager

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

#include "device_manager.h"
#include "bridges/device_bridge.h"
#include "bridges/telescope_bridge.h"
#include "alpaca_client.h"
#include "indilogger.h"

#include <httplib.h>
#include <cstring>
#include <sstream>

#ifdef _USE_SYSTEM_JSONLIB
#include <nlohmann/json.hpp>
#else
#include <indijson.hpp>
#endif

using json = nlohmann::json;

// Static instance for singleton
DeviceManager* DeviceManager::getInstance()
{
    static DeviceManager instance;
    return &instance;
}

DeviceManager::DeviceManager()
{
    DEBUGDEVICE("Device Manager", INDI::Logger::DBG_SESSION, "Device manager initialized");
}

DeviceManager::~DeviceManager()
{
    DEBUGDEVICE("Device Manager", INDI::Logger::DBG_SESSION, "Device manager destroyed");
}

void DeviceManager::setAlpacaClient(const std::shared_ptr<AlpacaClient> &client)
{
    m_Client = client;
    DEBUGDEVICE("Device Manager", INDI::Logger::DBG_SESSION, "AlpacaClient set");
}

void DeviceManager::sendNewNumber(const INDI::PropertyNumber &numberProperty)
{
    if (m_Client)
    {
        DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_DEBUG, "Sending new number property: %s", numberProperty.getName());
        m_Client->sendNewNumber(numberProperty);
    }
    else
    {
        DEBUGDEVICE("Device Manager", INDI::Logger::DBG_ERROR, "Cannot send new number property: AlpacaClient not set");
    }
}

void DeviceManager::sendNewSwitch(const INDI::PropertySwitch &switchProperty)
{
    if (m_Client)
    {
        DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_DEBUG, "Sending new switch property: %s", switchProperty.getName());
        m_Client->sendNewSwitch(switchProperty);
    }
    else
    {
        DEBUGDEVICE("Device Manager", INDI::Logger::DBG_ERROR, "Cannot send new switch property: AlpacaClient not set");
    }
}

void DeviceManager::addDevice(INDI::BaseDevice device)
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    const char *deviceName = device.getDeviceName();
    DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_SESSION, "Adding device: %s", deviceName);

    // Check if device already exists
    if (m_Devices.find(deviceName) != m_Devices.end())
    {
        DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_SESSION, "Device %s already exists, updating", deviceName);
        m_Devices[deviceName] = device;
        return;
    }

    // Add device to map
    m_Devices[deviceName] = device;

    // Create bridge for device
    int deviceNumber = m_NextDeviceNumber++;
    auto bridge = createBridge(device, deviceNumber);
    if (bridge)
    {
        m_Bridges[deviceNumber] = std::move(bridge);
        m_DeviceNumberMap[deviceName] = deviceNumber;
        DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_SESSION, "Created bridge for device %s with number %d", deviceName,
                     deviceNumber);
    }
    else
    {
        DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_ERROR, "Failed to create bridge for device %s", deviceName);
    }
}

void DeviceManager::removeDevice(INDI::BaseDevice device)
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    const char *deviceName = device.getDeviceName();
    DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_SESSION, "Removing device: %s", deviceName);

    // Check if device exists
    auto it = m_DeviceNumberMap.find(deviceName);
    if (it == m_DeviceNumberMap.end())
    {
        DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_SESSION, "Device %s not found", deviceName);
        return;
    }

    // Remove bridge
    int deviceNumber = it->second;
    m_Bridges.erase(deviceNumber);
    m_DeviceNumberMap.erase(it);
    m_Devices.erase(deviceName);

    DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_SESSION, "Removed device %s with number %d", deviceName, deviceNumber);
}

void DeviceManager::updateDeviceProperty(INDI::Property property)
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    const char *deviceName = property.getDeviceName();
    DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_DEBUG, "Updating property for device %s: %s", deviceName,
                 property.getName());

    // Find device number
    auto it = m_DeviceNumberMap.find(deviceName);
    if (it == m_DeviceNumberMap.end())
    {
        DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_DEBUG, "Device %s not found for property update", deviceName);
        return;
    }

    // Update bridge
    int deviceNumber = it->second;
    auto bridgeIt = m_Bridges.find(deviceNumber);
    if (bridgeIt != m_Bridges.end())
    {
        bridgeIt->second->updateProperty(property);
    }
}

std::unique_ptr<IDeviceBridge> DeviceManager::createBridge(INDI::BaseDevice device, int deviceNumber)
{
    // Check device interface to determine type
    uint32_t interface = device.getDriverInterface();

    // Create appropriate bridge based on interface
    if (interface & INDI::BaseDevice::TELESCOPE_INTERFACE)
    {
        DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_SESSION, "Creating telescope bridge for device %s",
                     device.getDeviceName());
        return std::make_unique<TelescopeBridge>(device, deviceNumber);
    }
    // Add more device types here as they are implemented
    // else if (interface & INDI::BaseDevice::CCD_INTERFACE)
    // {
    //     DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_SESSION, "Creating camera bridge for device %s", device.getDeviceName());
    //     return std::make_unique<CameraBridge>(device, deviceNumber);
    // }
    // else if (interface & INDI::BaseDevice::DOME_INTERFACE)
    // {
    //     DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_SESSION, "Creating dome bridge for device %s", device.getDeviceName());
    //     return std::make_unique<DomeBridge>(device, deviceNumber);
    // }

    DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_WARNING, "Unsupported device interface: %u for device %s", interface,
                 device.getDeviceName());
    return nullptr;
}

void DeviceManager::handleAlpacaRequest(const httplib::Request &req, httplib::Response &res)
{
    DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_DEBUG, "Handling Alpaca request: %s", req.path.c_str());

    // Parse path to determine if it's a management or device API request
    std::string path = req.path;

    // Management API
    if (path.find("/management/") == 0)
    {
        handleManagementRequest(path.substr(12), req, res);
        return;
    }

    // Device API
    if (path.find("/api/v1/") == 0)
    {
        std::string apiPath = path.substr(8); // Remove "/api/v1/"

        // Parse device type, number, and method
        std::istringstream ss(apiPath);
        std::string deviceType, deviceNumberStr, method;

        std::getline(ss, deviceType, '/');
        std::getline(ss, deviceNumberStr, '/');
        std::getline(ss, method, '/');

        // Validate
        if (deviceType.empty() || deviceNumberStr.empty() || method.empty())
        {
            json response =
            {
                {"ErrorNumber", 1001},
                {"ErrorMessage", "Invalid API request format"}
            };
            res.set_content(response.dump(), "application/json");
            return;
        }

        // Convert device number
        int deviceNumber;
        try
        {
            deviceNumber = std::stoi(deviceNumberStr);
        }
        catch (const std::exception &e)
        {
            json response =
            {
                {"ErrorNumber", 1002},
                {"ErrorMessage", "Invalid device number"}
            };
            res.set_content(response.dump(), "application/json");
            return;
        }

        // Route to appropriate device
        routeRequest(deviceNumber, deviceType, method, req, res);
        return;
    }

    // Unknown API
    json response =
    {
        {"ErrorNumber", 1000},
        {"ErrorMessage", "Unknown API endpoint"}
    };
    res.set_content(response.dump(), "application/json");
}

void DeviceManager::routeRequest(int deviceNumber, const std::string &deviceType,
                                 const std::string &method,
                                 const httplib::Request &req, httplib::Response &res)
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    // Find bridge for device number
    auto it = m_Bridges.find(deviceNumber);
    if (it == m_Bridges.end())
    {
        json response =
        {
            {"ErrorNumber", 1003},
            {"ErrorMessage", "Device not found"}
        };
        res.set_content(response.dump(), "application/json");
        return;
    }

    // Check if device type matches
    if (it->second->getDeviceType() != deviceType)
    {
        json response =
        {
            {"ErrorNumber", 1004},
            {"ErrorMessage", "Device type mismatch"}
        };
        res.set_content(response.dump(), "application/json");
        return;
    }

    // Forward request to bridge
    it->second->handleRequest(method, req, res);
}

void DeviceManager::handleManagementRequest(const std::string &endpoint,
        const httplib::Request &req,
        httplib::Response &res)
{
    INDI_UNUSED(req);
    DEBUGFDEVICE("Device Manager", INDI::Logger::DBG_DEBUG, "Handling management request: %s", endpoint.c_str());

    if (endpoint == "apiversions")
    {
        // Return supported API versions
        json response =
        {
            {"Value", json::array({1})},
            {"ClientTransactionID", 0},
            {"ServerTransactionID", 0},
            {"ErrorNumber", 0},
            {"ErrorMessage", ""}
        };
        res.set_content(response.dump(), "application/json");
    }
    else if (endpoint == "v1/description")
    {
        // Return server description
        json response =
        {
            {"Value", "INDI Alpaca Server"},
            {"ClientTransactionID", 0},
            {"ServerTransactionID", 0},
            {"ErrorNumber", 0},
            {"ErrorMessage", ""}
        };
        res.set_content(response.dump(), "application/json");
    }
    else if (endpoint == "v1/configureddevices")
    {
        // Return list of configured devices
        json devices = json::array();

        std::lock_guard<std::mutex> lock(m_Mutex);
        for (const auto &pair : m_Bridges)
        {
            devices.push_back(
            {
                {"DeviceName", pair.second->getDeviceName()},
                {"DeviceType", pair.second->getDeviceType()},
                {"DeviceNumber", pair.second->getDeviceNumber()},
                {"UniqueID", pair.second->getUniqueID()}
            });
        }

        json response =
        {
            {"Value", devices},
            {"ClientTransactionID", 0},
            {"ServerTransactionID", 0},
            {"ErrorNumber", 0},
            {"ErrorMessage", ""}
        };
        res.set_content(response.dump(), "application/json");
    }
    else
    {
        // Unknown management endpoint
        json response =
        {
            {"ErrorNumber", 1005},
            {"ErrorMessage", "Unknown management endpoint"}
        };
        res.set_content(response.dump(), "application/json");
    }
}

std::vector<AlpacaDeviceInfo> DeviceManager::getDeviceList()
{
    std::lock_guard<std::mutex> lock(m_Mutex);

    std::vector<AlpacaDeviceInfo> devices;
    for (const auto &pair : m_Bridges)
    {
        AlpacaDeviceInfo info;
        info.deviceNumber = pair.second->getDeviceNumber();
        info.deviceName = pair.second->getDeviceName();
        info.deviceType = pair.second->getDeviceType();
        info.uniqueID = pair.second->getUniqueID();
        devices.push_back(info);
    }

    return devices;
}
