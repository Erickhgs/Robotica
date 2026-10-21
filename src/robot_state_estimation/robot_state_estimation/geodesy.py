"""Conversao WGS84 -> ECEF -> ENU, sem dependencias de ROS.

ENU e um plano tangente com origem fixa conhecida, em metros.
Nao use a primeira amostra ruidosa de GPS como origem de cada ensaio.
"""
import math


def geodetic_to_ecef(latitude, longitude, altitude):
    if not all(math.isfinite(v) for v in (latitude, longitude, altitude)):
        raise ValueError('Coordenadas devem ser finitas.')
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        raise ValueError('Latitude/longitude fora do intervalo em graus.')
    a, e2 = 6378137.0, 6.6943799901413165e-3
    lat, lon = math.radians(latitude), math.radians(longitude)
    n = a / math.sqrt(1.0 - e2 * math.sin(lat)**2)
    return ((n + altitude)*math.cos(lat)*math.cos(lon),
            (n + altitude)*math.cos(lat)*math.sin(lon),
            (n*(1.0-e2) + altitude)*math.sin(lat))


class LocalENU:
    def __init__(self, latitude, longitude, altitude):
        self.origin = geodetic_to_ecef(latitude, longitude, altitude)
        lat, lon = math.radians(latitude), math.radians(longitude)
        self.sl, self.cl = math.sin(lat), math.cos(lat)
        self.so, self.co = math.sin(lon), math.cos(lon)

    def convert(self, latitude, longitude, altitude):
        xyz = geodetic_to_ecef(latitude, longitude, altitude)
        dx, dy, dz = (a-b for a, b in zip(xyz, self.origin))
        return (-self.so*dx + self.co*dy,
                -self.sl*self.co*dx - self.sl*self.so*dy + self.cl*dz,
                self.cl*self.co*dx + self.cl*self.so*dy + self.sl*dz)


def normalized_quaternion(x, y, z, w):
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Quaternion invalido.')
    return tuple(v/norm for v in (x, y, z, w))


def yaw_from_quaternion(x, y, z, w):
    x, y, z, w = normalized_quaternion(x, y, z, w)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def add_world_yaw(x, y, z, w, delta):
    """q_z(delta) * q; preserva roll/pitch e mantem quaternion unitario."""
    if not math.isfinite(delta):
        raise ValueError('Ruido angular invalido.')
    x, y, z, w = normalized_quaternion(x, y, z, w)
    c, s = math.cos(delta/2), math.sin(delta/2)
    return c*x-s*y, c*y+s*x, c*z+s*w, c*w-s*z


def fresh_pair(now, gps_stamp, imu_stamp, max_age, max_skew):
    values = (now, gps_stamp, imu_stamp, max_age, max_skew)
    return (all(math.isfinite(v) for v in values) and max_age > 0 and max_skew >= 0
            and 0 <= now-gps_stamp <= max_age and 0 <= now-imu_stamp <= max_age
            and abs(gps_stamp-imu_stamp) <= max_skew)
