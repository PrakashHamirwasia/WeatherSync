# 🌦️ WeatherSync

## Smart IoT-Based Weather Monitoring Station

WeatherSync is an **IoT-enabled weather monitoring system** designed to collect, process, store, and visualize localized environmental data. The system uses an **ESP32-S3** as the primary controller and integrates multiple sensors to measure important meteorological parameters such as temperature, humidity, atmospheric pressure, UV radiation, rainfall, wind speed, and wind direction.

The collected data is transmitted to a cloud-based backend and can be accessed through the **WeatherSync Android application**, enabling remote monitoring and analysis of current and historical weather conditions.

---

## 🎯 Project Objective

The primary objective of WeatherSync is to develop a **cost-effective, scalable, and localized weather monitoring station** capable of continuously collecting environmental data and making it remotely accessible.

The system is intended to provide reliable site-specific weather information for applications such as **agriculture, environmental monitoring, education, research, and smart-campus deployments**.

---

## ✨ Features

- 🌡️ Temperature monitoring
- 💧 Relative humidity monitoring
- 🌡️ Atmospheric pressure measurement
- ☀️ UV radiation monitoring
- 🌧️ Rainfall measurement
- 🌬️ Wind speed measurement
- 🧭 Wind direction measurement
- 📡 IoT-based data communication
- ☁️ Cloud-based data storage
- 📱 Android application for remote monitoring
- 📊 Historical weather-data visualization
- 🛡️ Weather-protected sensor enclosure
- 🌿 3D-printed Gill radiation shield
- 🔄 Periodic weather-data collection
- 👥 Multi-user access to station data

> **Future:** Solar energy integration and LoRa-based communication will be added in future updates.

---

# 🏗️ System Architecture

```text
                    ┌──────────────────────────┐
                    │      Weather Sensors     │
                    │                          │
                    │  • SHT85                 │
                    │  • BMP390                │
                    │  • VEML6070              │
                    │  • Rain Gauge            │
                    │  • Wind Speed Sensor     │
                    │  • Wind Direction Sensor │
                    └────────────┬─────────────┘
                                 │
                                 ▼
                    ┌──────────────────────────┐
                    │        ESP32-S3           │
                    │                          │
                    │  • Sensor Acquisition    │
                    │  • Data Processing       │
                    │  • Communication         │
                    └────────────┬─────────────┘
                                 │
                                 ▼
                    ┌──────────────────────────┐
                    │       MQTT / HiveMQ       │
                    └────────────┬─────────────┘
                                 │
                                 ▼
                    ┌──────────────────────────┐
                    │         Supabase          │
                    │     Cloud Data Storage    │
                    └────────────┬─────────────┘
                                 │
                                 ▼
                    ┌──────────────────────────┐
                    │     WeatherSync App       │
                    │         Flutter           │
                    └──────────────────────────┘
