import math

# Basit enerji tüketimi modeli ile yük, hız, rüzgâr ve irtifa etkileri

def compute_energy(distance: float,
                   payload_weight: float,
                   speed: float,
                   wind_speed: float = 0.0,
                   elevation_gain: float = 0.0) -> float:
    """
    Mesafe (m), yük (kg), hız (m/s), rüzgâr hızı (m/s) ve irtifa kazanımı (m) parametrelerine göre
    tahmini enerji tüketimi (Wh) döner.
    Formül:
      P_hover = 200.0  # sabit hover gücü (W)
      P_payload = 20.0 * payload_weight  # yük artış gücü (W)
      P_wind = 50.0 * abs(wind_speed)  # rüzgâr cezası (W)
      P_climb = 9.81 * payload_weight * climb_rate / 3600  # irtifa tırmanışı (W)
      time_h = distance / speed / 3600  # saat cinsine çevir
      E = (P_hover + P_payload + P_wind) * time_h + P_climb * (elevation_gain / climb_rate)
    Baskı değerleri örnektir.
    """
    # Sabitler
    P_hover = 200.0
    P_payload = 20.0 * payload_weight
    P_wind = 50.0 * abs(wind_speed)
    # Varsayılan tırmanma hızı (m/s)
    climb_rate = 1.0
    # Tırmanış gücü
    P_climb = 9.81 * payload_weight * climb_rate / 3600.0
    # zaman (saat)
    time_h = distance / speed / 3600.0
    # Enerji (Wh)
    E = (P_hover + P_payload + P_wind) * time_h + P_climb * (elevation_gain / climb_rate)
    return E 