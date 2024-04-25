"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.

Type annotations for the sensor client python bindings.

This is a mypy stub file defining just the type signatures of the module
generated by pybind11. It was generated using the ``stubgen`` utility and then
modified.

Note:
    This file should be updated whenever the bindings are modified.

"""
# flake8: noqa (linter complains about scoping, but afaict mypy doesn't care)

from numpy import ndarray
from typing import (Callable, ClassVar, Dict, Iterator, List, Optional,
                    overload, Tuple, Union)

from .data import (BufferT, ColHeader, FieldDType)


class Client:
    @overload
    def __init__(self,
                 hostname: str = ...,
                 lidar_port: int = ...,
                 imu_port: int = ...,
                 capacity: int = ...) -> None:
        ...

    @overload
    def __init__(self,
                 hostname: str,
                 udp_dest_host: str,
                 mode: LidarMode = ...,
                 timestamp_mode: TimestampMode = ...,
                 lidar_port: int = ...,
                 imu_port: int = ...,
                 timeout_sec: int = ...,
                 capacity: int = ...) -> None:
        ...

    def get_metadata(self, timeout_sec: int = ..., legacy: bool = ...) -> str:
        ...

    def shutdown(self) -> None:
        ...

    def consume(self, buf: bytearray, timeout_sec: float) -> ClientState:
        ...

    def produce(self, pf: PacketFormat) -> None:
        ...

    def flush(self, n_packets: int = ...) -> None:
        ...

    @property
    def capacity(self) -> int:
        ...

    @property
    def size(self) -> int:
        ...

    @property
    def lidar_port(self) -> int:
        ...

    @property
    def imu_port(self) -> int:
        ...


class ClientState:
    ERROR: ClassVar[ClientState]
    EXIT: ClassVar[ClientState]
    IMU_DATA: ClassVar[ClientState]
    LIDAR_DATA: ClassVar[ClientState]
    TIMEOUT: ClassVar[ClientState]
    OVERFLOW: ClassVar[ClientState]

    __members__: ClassVar[Dict[str, ClientState]]

    def __init__(self, x: int) -> None:
        ...

    def __and__(self, s: ClientState) -> int:
        ...

    def __int__(self) -> int:
        ...

    def __invert__(self) -> int:
        ...

    def __or__(self, s: ClientState) -> int:
        ...

    def __xor__(self, s: ClientState) -> int:
        ...


class SensorInfo:
    hostname: str
    sn: str
    fw_rev: str
    mode: LidarMode
    prod_line: str
    format: DataFormat
    beam_azimuth_angles: List[float]
    beam_altitude_angles: List[float]
    imu_to_sensor_transform: ndarray
    lidar_to_sensor_transform: ndarray
    lidar_origin_to_beam_origin_mm: float
    beam_to_lidar_transform: ndarray
    extrinsic: ndarray
    init_id: int
    udp_port_lidar: int
    udp_port_imu: int

    @classmethod
    def from_default(cls, mode: LidarMode) -> SensorInfo:
        ...

    @overload
    def __init__(self) -> None:
        ...

    @overload
    def __init__(self, metadata: str) -> None:
        ...


class DataFormat:
    columns_per_frame: int
    columns_per_packet: int
    pixel_shift_by_row: List[int]
    pixels_per_column: int
    column_window: Tuple[int, int]
    udp_profile_lidar: UDPProfileLidar
    udp_profile_imu: UDPProfileIMU
    fps: int


class PacketFormat:
    @property
    def lidar_packet_size(self) -> int:
        ...

    @property
    def imu_packet_size(self) -> int:
        ...

    @property
    def columns_per_packet(self) -> int:
        ...

    @property
    def pixels_per_column(self) -> int:
        ...

    def packet_type(self, buf: BufferT) -> int:
        ...

    def frame_id(self, buf: BufferT) -> int:
        ...

    def prod_sn(self, buf: BufferT) -> int:
        ...

    def init_id(self, buf: BufferT) -> int:
        ...

    def countdown_thermal_shutdown(self, buf: BufferT) -> int:
        ...

    def countdown_shot_limiting(self, buf: BufferT) -> int:
        ...

    def thermal_shutdown(self, buf: BufferT) -> int:
        ...

    def shot_limiting(self, buf: BufferT) -> int:
        ...

    @property
    def fields(self) -> Iterator[ChanField]:
        ...

    def packet_field(self, field: ChanField, buf: BufferT) -> ndarray:
        ...

    def packet_header(self, header: ColHeader, buf: BufferT) -> ndarray:
        ...

    def imu_sys_ts(self, buf: BufferT) -> int:
        ...

    def imu_accel_ts(self, buf: BufferT) -> int:
        ...

    def imu_gyro_ts(self, buf: BufferT) -> int:
        ...

    def imu_av_x(self, buf: BufferT) -> float:
        ...

    def imu_av_y(self, buf: BufferT) -> float:
        ...

    def imu_av_z(self, buf: BufferT) -> float:
        ...

    def imu_la_x(self, buf: BufferT) -> float:
        ...

    def imu_la_y(self, buf: BufferT) -> float:
        ...

    def imu_la_z(self, buf: BufferT) -> float:
        ...

    @staticmethod
    def from_info(info: SensorInfo) -> PacketFormat:
        ...


class LidarMode:
    MODE_UNSPEC: ClassVar[LidarMode]
    MODE_512x10: ClassVar[LidarMode]
    MODE_512x20: ClassVar[LidarMode]
    MODE_1024x10: ClassVar[LidarMode]
    MODE_1024x20: ClassVar[LidarMode]
    MODE_2048x10: ClassVar[LidarMode]
    MODE_4096x5: ClassVar[LidarMode]

    __members__: ClassVar[Dict[str, LidarMode]]
    values: ClassVar[Iterator[LidarMode]]

    def __init__(self, code: int) -> None:
        ...

    def __int__(self) -> int:
        ...

    @property
    def cols(self) -> int:
        ...

    @property
    def frequency(self) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

    @classmethod
    def from_string(cls, s: str) -> LidarMode:
        ...


class TimestampMode:
    TIME_FROM_UNSPEC: ClassVar[TimestampMode]
    TIME_FROM_INTERNAL_OSC: ClassVar[TimestampMode]
    TIME_FROM_PTP_1588: ClassVar[TimestampMode]
    TIME_FROM_SYNC_PULSE_IN: ClassVar[TimestampMode]

    __members__: ClassVar[Dict[str, TimestampMode]]
    values: ClassVar[Iterator[TimestampMode]]

    def __init__(self, code: int) -> None:
        ...

    def __int__(self) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

    @classmethod
    def from_string(cls, s: str) -> TimestampMode:
        ...


class OperatingMode:
    OPERATING_NORMAL: ClassVar[OperatingMode]
    OPERATING_STANDBY: ClassVar[OperatingMode]

    __members__: ClassVar[Dict[str, OperatingMode]]
    values: ClassVar[Iterator[OperatingMode]]

    def __init__(self, code: int) -> None:
        ...

    def __int__(self) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

    @classmethod
    def from_string(cls, s: str) -> OperatingMode:
        ...


class MultipurposeIOMode:
    MULTIPURPOSE_OFF: ClassVar[MultipurposeIOMode]
    MULTIPURPOSE_INPUT_NMEA_UART: ClassVar[MultipurposeIOMode]
    MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC: ClassVar[MultipurposeIOMode]
    MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN: ClassVar[MultipurposeIOMode]
    MULTIPURPOSE_OUTPUT_FROM_PTP_1588: ClassVar[MultipurposeIOMode]
    MULTIPURPOSE_OUTPUT_FROM_ENCODER_ANGLE: ClassVar[MultipurposeIOMode]

    __members__: ClassVar[Dict[str, MultipurposeIOMode]]
    values: ClassVar[Iterator[MultipurposeIOMode]]

    def __init__(self, code: int) -> None:
        ...

    def __int__(self) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

    @classmethod
    def from_string(cls, s: str) -> MultipurposeIOMode:
        ...


class Polarity:
    POLARITY_ACTIVE_HIGH: ClassVar[Polarity]
    POLARITY_ACTIVE_LOW: ClassVar[Polarity]

    __members__: ClassVar[Dict[str, Polarity]]
    values: ClassVar[Iterator[Polarity]]

    def __init__(self, code: int) -> None:
        ...

    def __int__(self) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

    @classmethod
    def from_string(cls, s: str) -> Polarity:
        ...


class NMEABaudRate:
    BAUD_9600: ClassVar[NMEABaudRate]
    BAUD_115200: ClassVar[NMEABaudRate]

    __members__: ClassVar[Dict[str, NMEABaudRate]]
    values: ClassVar[Iterator[NMEABaudRate]]

    def __init__(self, code: int) -> None:
        ...

    def __int__(self) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

    @classmethod
    def from_string(cls, s: str) -> NMEABaudRate:
        ...


class ChanField:
    RANGE: ClassVar[ChanField]
    RANGE2: ClassVar[ChanField]
    SIGNAL: ClassVar[ChanField]
    SIGNAL2: ClassVar[ChanField]
    REFLECTIVITY: ClassVar[ChanField]
    REFLECTIVITY2: ClassVar[ChanField]
    FLAGS: ClassVar[ChanField]
    FLAGS2: ClassVar[ChanField]
    NEAR_IR: ClassVar[ChanField]
    RAW_HEADERS: ClassVar[ChanField]
    CUSTOM0: ClassVar[ChanField]
    CUSTOM1: ClassVar[ChanField]
    CUSTOM2: ClassVar[ChanField]
    CUSTOM3: ClassVar[ChanField]
    CUSTOM4: ClassVar[ChanField]
    CUSTOM5: ClassVar[ChanField]
    CUSTOM6: ClassVar[ChanField]
    CUSTOM7: ClassVar[ChanField]
    CUSTOM8: ClassVar[ChanField]
    CUSTOM9: ClassVar[ChanField]
    RAW32_WORD1: ClassVar[ChanField]
    RAW32_WORD2: ClassVar[ChanField]
    RAW32_WORD3: ClassVar[ChanField]
    RAW32_WORD4: ClassVar[ChanField]
    RAW32_WORD5: ClassVar[ChanField]
    RAW32_WORD6: ClassVar[ChanField]
    RAW32_WORD7: ClassVar[ChanField]
    RAW32_WORD8: ClassVar[ChanField]
    RAW32_WORD9: ClassVar[ChanField]

    __members__: ClassVar[Dict[str, ChanField]]
    values: ClassVar[Iterator[ChanField]]

    def __init__(self, code: int) -> None:
        ...

    def __int__(self) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

    @classmethod
    def from_string(cls, s: str) -> ChanField:
        ...


class UDPProfileLidar:
    PROFILE_LIDAR_LEGACY: ClassVar[UDPProfileLidar]
    PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL: ClassVar[UDPProfileLidar]
    PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16: ClassVar[UDPProfileLidar]
    PROFILE_LIDAR_RNG15_RFL8_NIR8: ClassVar[UDPProfileLidar]
    PROFILE_LIDAR_FIVE_WORD_PIXEL: ClassVar[UDPProfileLidar]

    __members__: ClassVar[Dict[str, UDPProfileLidar]]
    values: ClassVar[Iterator[UDPProfileLidar]]

    def __init__(self, code: int) -> None:
        ...

    def __int__(self) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

    @classmethod
    def from_string(cls, s: str) -> UDPProfileLidar:
        ...


class UDPProfileIMU:
    PROFILE_IMU_LEGACY: ClassVar[UDPProfileIMU]

    __members__: ClassVar[Dict[str, UDPProfileIMU]]
    values: ClassVar[Iterator[UDPProfileIMU]]

    def __init__(self, code: int) -> None:
        ...

    def __int__(self) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

    @classmethod
    def from_string(cls, s: str) -> UDPProfileIMU:
        ...


class SensorConfig:
    udp_dest: Optional[str]
    udp_port_lidar: Optional[int]
    udp_port_imu: Optional[int]
    timestamp_mode: Optional[TimestampMode]
    lidar_mode: Optional[LidarMode]
    operating_mode: Optional[OperatingMode]
    multipurpose_io_mode: Optional[MultipurposeIOMode]
    azimuth_window: Optional[tuple]
    signal_multiplier: Optional[float]
    sync_pulse_out_angle: Optional[int]
    sync_pulse_out_pulse_width: Optional[int]
    nmea_in_polarity: Optional[Polarity]
    nmea_baud_rate: Optional[NMEABaudRate]
    nmea_ignore_valid_char: Optional[bool]
    nmea_leap_seconds: Optional[int]
    sync_pulse_in_polarity: Optional[Polarity]
    sync_pulse_out_polarity: Optional[Polarity]
    sync_pulse_out_frequency: Optional[int]
    phase_lock_enable: Optional[bool]
    phase_lock_offset: Optional[int]
    columns_per_packet: Optional[int]
    udp_profile_lidar: Optional[UDPProfileLidar]
    udp_profile_imu: Optional[UDPProfileIMU]

    @overload
    def __init__(self) -> None:
        ...

    @overload
    def __init__(self, config_string: str) -> None:
        ...

def convert_to_legacy(metadata: str) -> str:
    ...

def init_logger(log_level: str,
                log_file_path: str = ...,
                rotating: bool = ...,
                max_size_in_bytes: int = ...,
                max_files: int = ...) -> bool:
    ...


def set_config(hostname: str,
               config: SensorConfig,
               persist: bool = ...,
               udp_dest_auto: bool = ...) -> None:
    ...


def get_config(hostname: str, active: bool = ...) -> SensorConfig:
    ...


class Version:
    major: int
    minor: int
    patch: int

    def __init__(self) -> None:
        ...

    def __le__(self, v: Version) -> bool:
        ...

    def __lt__(self, v: Version) -> bool:
        ...

    @classmethod
    def from_string(cls, s: str) -> Version:
        ...


class LidarScan:

    frame_id: int
    frame_status: int

    @overload
    def __init__(self, w: int, h: int) -> None:
        ...

    @overload
    def __init__(self, h: int, w: int, profile: UDPProfileLidar) -> None:
        ...

    @overload
    def __init__(self, h: int, w: int, fields: Dict[ChanField,
                                                    FieldDType]) -> None:
        ...

    @property
    def w(self) -> int:
        ...

    @property
    def h(self) -> int:
        ...


    def thermal_shutdown(self) -> int:
        ...

    def shot_limiting(self) -> int:
        ...

    def field(self, field: ChanField) -> ndarray:
        ...

    @property
    def timestamp(self) -> ndarray:
        ...

    @property
    def measurement_id(self) -> ndarray:
        ...

    @property
    def status(self) -> ndarray:
        ...

    def complete(self, window: Optional[Tuple[int, int]] = ...) -> bool:
        ...

    @property
    def fields(self) -> Iterator[ChanField]:
        ...

    def to_native(self) -> LidarScan:
        ...

    @classmethod
    def from_native(cls, scan: LidarScan) -> LidarScan:
        ...


def destagger_int8(field: ndarray, shifts: List[int],
                   inverse: bool) -> ndarray:
    ...


def destagger_int16(field: ndarray, shifts: List[int],
                    inverse: bool) -> ndarray:
    ...


def destagger_int32(field: ndarray, shifts: List[int],
                    inverse: bool) -> ndarray:
    ...


def destagger_int64(field: ndarray, shifts: List[int],
                    inverse: bool) -> ndarray:
    ...


def destagger_uint8(field: ndarray, shifts: List[int],
                    inverse: bool) -> ndarray:
    ...


def destagger_uint16(field: ndarray, shifts: List[int],
                     inverse: bool) -> ndarray:
    ...


def destagger_uint32(field: ndarray, shifts: List[int],
                     inverse: bool) -> ndarray:
    ...


def destagger_uint64(field: ndarray, shifts: List[int],
                     inverse: bool) -> ndarray:
    ...


def destagger_float(field: ndarray, shifts: List[int],
                    inverse: bool) -> ndarray:
    ...


def destagger_double(field: ndarray, shifts: List[int],
                     inverse: bool) -> ndarray:
    ...


class ScanBatcher:
    @overload
    def __init__(self, w: int, pf: PacketFormat) -> None:
        ...

    @overload
    def __init__(self, info: SensorInfo) -> None:
        ...

    def __call__(self, buf: BufferT, ls: LidarScan) -> bool:
        ...


class XYZLut:
    def __init__(self, info: SensorInfo, use_extrinsics: bool) -> None:
        ...

    @overload
    def __call__(self, scan: LidarScan) -> ndarray:
        ...

    @overload
    def __call__(self, range: ndarray) -> ndarray:
        ...


class AutoExposure:
    @overload
    def __init__(self) -> None:
        ...

    @overload
    def __init__(self, lo_percentile: float, hi_percentile: float,
                 update_every: int) -> None:
        ...

    def __call__(self, image: ndarray, update_state: Optional[bool] = True) -> None:
        ...


class BeamUniformityCorrector:
    def __init__(self) -> None:
        ...

    def __call__(self, image: ndarray) -> None:
        ...

class Imu:
    @overload
    def __init__(self, buf: BufferT, pf) -> None: ...

    @overload
    def __init__(self, accel: ndarray, angular_vel: ndarray,
                 sys_ts: int = ..., accel_ts: int = ..., gyro_ts: int = ...) -> None: ...

    @property
    def accel(self) -> ndarray: ...
    @property
    def accel_ts(self) -> int: ...
    @property
    def angular_vel(self) -> ndarray: ...
    @property
    def gyro_ts(self) -> int: ...
    @property
    def sys_ts(self) -> int: ...
