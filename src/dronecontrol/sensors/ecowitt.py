""" Sensor package for an ecowitt gw1100"""
import asyncio

import requests
import math
import datetime

from dronecontrol.plugins.sensor import Sensor

# Leitstand IP: 192.168.1.41
# Use HTTP GET with http://192.168.1.41/get_livedata_info, see https://blog.meteodrenthe.nl/2023/02/03/how-to-use-the-ecowitt-gateway-gw1000-gw1100-local-api/#
# Example output JSON: { "common_list": [{ "id": "0x02", "val": "22.9", "unit": "C" }, { "id": "0x07", "val": "41%" }, { "id": "3", "val": "22.9", "unit": "C" }, { "id": "0x03", "val": "9.0", "unit": "C" }, { "id": "0x0B", "val": "0.4 m/s" }, { "id": "0x0C", "val": "0.5 m/s" }, { "id": "0x19", "val": "1.0 m/s" }, { "id": "0x15", "val": "4.02 W/m2" }, { "id": "0x17", "val": "0" }, { "id": "0x0A", "val": "260" }], "rain": [{ "id": "0x0D", "val": "0.0 mm" }, { "id": "0x0E", "val": "0.0 mm/Hr" }, { "id": "0x10", "val": "0.0 mm" }, { "id": "0x11", "val": "0.0 mm" }, { "id": "0x12", "val": "0.0 mm" }, { "id": "0x13", "val": "0.0 mm", "battery": "0" }], "wh25": [{ "intemp": "22.9", "unit": "C", "inhumi": "42%", "abs": "975.8 hPa", "rel": "975.8 hPa" }] }

ECOWITT_ID_MAP_COMMON = {
    "0x02": "temperature",
    "0x03": "dew_point",
    "0x07": "humidity",
    "0x15": "light",
    "0x17": "uvi",
    "0x19": "wind_speed_max_day",
    "0x0A": "wind_direction",
    "0x0B": "wind_speed",
    "0x0C": "gust_speed",
}


ECOWITT_ID_MAP_RAIN = {
    "0x0D": "rain_event",
    "0x0E": "rain_rate",
    "0x10": "cum_rain_today",
    "0x11": "cum_rain_week",
    "0x12": "cum_rain_month",
    "0x13": "cum_rain_year",
}


class EcoWittDataEntry:
    def __init__(self, entry_id: str, name: str, value: float = math.nan, unit: str = ""):
        self.entry_id = entry_id
        self.name = name
        self.value = value
        self.unit = unit


class EcoWittData:

    def __init__(self, timestamp = None):
        self.temperature = EcoWittDataEntry("0x02", "Temperature")
        self.dew_point = EcoWittDataEntry("0x03", "Dew point")
        self.humidity = EcoWittDataEntry("0x07", "Humidity")
        self.light = EcoWittDataEntry("0x15", "Light")
        self.uvi = EcoWittDataEntry("0x17", "UVI")
        self.wind_speed = EcoWittDataEntry("0x0B", "Wind Speed")
        self.wind_direction = EcoWittDataEntry("0x0A", "Wind Direction")
        self.gust_speed = EcoWittDataEntry("0x0C", "Gust Speed")
        self.wind_speed_max_day = EcoWittDataEntry("0x19", "Day max wind")

        self.rain_event = EcoWittDataEntry("0x0D", "Rain Event")
        self.rain_rate = EcoWittDataEntry("0x0E", "Current Rain Rate")
        self.cum_rain_today = EcoWittDataEntry("0x10", "Rain this Day")
        self.cum_rain_week = EcoWittDataEntry("0x11", "Rain this Week")
        self.cum_rain_month = EcoWittDataEntry("0x12", "Rain this Month")
        self.cum_rain_year = EcoWittDataEntry("0x13", "Rain Year")

        self.pressure = EcoWittDataEntry("", "Pressure")
        self.time = timestamp
        self.data_entries = [self.temperature, self.dew_point, self.humidity, self.light, self.uvi, self.wind_speed,
                     self.wind_direction, self.gust_speed, self.wind_speed_max_day, self.rain_event, self.rain_rate,
                     self.cum_rain_today, self.cum_rain_week, self.cum_rain_month, self.cum_rain_year, self.pressure]

    def __str__(self):
        return "\t".join([f"{entry.name}: {entry.value}{entry.unit}" for entry in self.data_entries]) + f"\t Time {self.time}"

    @classmethod
    def from_dict(cls, input_dict, timestamp = None):
        output = cls(timestamp=timestamp)
        # Parse common list entry
        if "common_list" in input_dict:
            for entry in input_dict["common_list"]:
                entry_id, entry_value, entry_unit = output.parse_entry(entry)
                if entry_id in ECOWITT_ID_MAP_COMMON:
                    attr_name = ECOWITT_ID_MAP_COMMON[entry_id]
                    data_entry = output.__getattribute__(attr_name)
                    data_entry.value = entry_value
                    data_entry.unit = entry_unit
        if "rain" in input_dict:
            for entry in input_dict["rain"]:
                entry_id, entry_value, entry_unit = output.parse_entry(entry)
                if entry_id in ECOWITT_ID_MAP_RAIN:
                    attr_name = ECOWITT_ID_MAP_RAIN[entry_id]
                    data_entry = output.__getattribute__(attr_name)
                    data_entry.value = entry_value
                    data_entry.unit = entry_unit
        if "wh25" in input_dict:
            entry = input_dict["wh25"][0]
            valunit = entry["abs"]
            value, unit = valunit.split(" ")
            output.pressure.value = value
            output.pressure.unit = unit
        return output

    def parse_entry(self, entry):
        entry_id = entry.get("id")
        entry_value = entry.get("val", math.nan)
        entry_unit = entry.get("unit", None)
        if entry_unit is None:  # Unit probably in value
            splits = entry_value.split(" ")
            if len(splits) == 1:  # Either no unit in value, or a percentage
                if str(entry_value).endswith("%"):
                    entry_value = entry_value[:-1]
                    entry_unit = "%"
                else:
                    entry_unit = ""
            elif len(splits) == 2:  # Second entry is probably the unit
                entry_unit = splits[1]
                entry_value = splits[0]
        entry_value = float(entry_value)
        return entry_id, entry_value, entry_unit


class EcoWittSensor(Sensor):
    """ Class for EcoWitt Weather stations that support their HTTP API."""

    def __init__(self, dm, logger, name="ecowitt"):
        super().__init__(dm, logger, name)
        self.ip = None
        self.last_data: EcoWittData | None = None
        self.time_since_ping = math.nan

    async def connect(self, ip: str):
        """ No connection procedure"""
        self.logger.info(f"Adding sensor with {ip}...")
        try:
            response = await asyncio.get_running_loop().run_in_executor(None, requests.get,
                                                                        f"http://{ip}/get_livedata_info")
        except Exception as e:
            self.logger.warning(f"No response from Ecowitt sensor at {ip}")
            self.logger.debug(repr(e), exc_info = True)
            return False
        self.ip = ip
        self.logger.info(f"Connected to sensor at {self.ip}")
        return True

    async def get_data(self) -> EcoWittData | None:
        try:
            timestamp = datetime.datetime.now(datetime.UTC)
            response = await asyncio.get_running_loop().run_in_executor(None, requests.get, f"http://{self.ip}/get_livedata_info")
            data = EcoWittData.from_dict(response.json(), timestamp=timestamp)
            self.last_data = data
            return data
        except Exception as e:
            self.logger.warning("Couldn't get a response from sensor!")
            self.logger.debug(repr(e), exc_info=True)
            return None

    async def status(self):
        """ No status as such to report. """
        self.logger.info(f"EcoWitt sensor as {self.PREFIX} with IP {self.ip}")

    async def disconnect(self):
        """ No disconnect procedure."""
        pass
