# Drone Teslimat Rota Planlayıcı

Bu proje, enerji kısıtları, dinamik no-fly zone'lar ve zaman pencereleri (TSPTW) dahil kompleks kısıtlar altında çalışan dronelar için teslimat rotalarını optimize eden bir Python uygulamasıdır.

## Özellikler

- **Veri Yapıları**: Drone, Teslimat Noktası ve No-Fly Zone tanımları (`dataclass` ile)
- **A\***: Tek-duraklı rota planlaması (zaman penceresi & no-fly cezaları)
- **TSPTW**: Çok-duraklı rota planlaması için DP tabanlı çözüm metodu
- **CSP**: Kısıt Programlama ile hızlı drone-teslimat ataması
- **Genetik Algoritma**: 2-opt local search entegrasyonu ile meta-heuristik optimizasyon
- **Enerji Modeli**: Yük, hız, rüzgâr ve irtifa etkilerine dayalı gerçekçi enerji tüketimi
- **Veri Üreticisi**: Rastgele drone/delivery/no-fly zone üretimi
- **Senaryo Testi**: `run_scenarios.py` ile örnek senaryolar (5 drone, 20 teslimat; 10 drone, 50 teslimat)
- **Streamlit Arayüzü**: `app.py` ile interaktif web tabanlı kontrol ve görselleştirme
- **FastAPI Servisi**: `server.py` ile HTTP ve WebSocket üzerinden dinamik planlama
- **Log Yönetimi**: Migration tarzı run-ID'lerle persistent log dosyaları

## Kurulum

```bash
git clone <repo-url>
cd <repo-directory>
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

**requirements.txt** örneği:

```
fastapi
uvicorn
streamlit
matplotlib
pydantic
```

## Kullanım

### 1. Komut Satırı Senaryoları

```bash
python run_scenarios.py
```

- `Senaryo1_csp.png`, `Senaryo2_csp.png` görselleştirmeleri oluşturur.
- Konsolda teslimat yüzdesi, enerji tüketimi, zaman pencere ihlali, ortalama bekleme süresi ve çalışma süreleri raporlanır.

### 2. Streamlit Arayüzü

```bash
streamlit run app.py
```

- Tarayıcıda dron sayısı, teslimat sayısı, no-fly zone, alan boyutu, CSP/GA seçeneklerini belirleyip "Çalıştır" butonuyla sonuçları log ve harita üzerinde izleyebilirsiniz.

### 3. FastAPI Dinamik Servis

```bash
uvicorn server:app --reload
```

- **POST /plan**: JSON payload ile CSP ve/veya GA planlamayı tetikler.
- **/docs**: Swagger UI dokümantasyon arayüzü.
- **WebSocket /ws**: `init`, `update_no_fly`, `new_delivery`, `replan` aksiyonlarıyla gerçek zamanlı planlama.

## Proje Yapısı

```
.
├── drone_routing/
│   ├── models.py           # Veri yapıları
│   ├── graph.py            # Graf, A* ve TSPTW metotları
│   ├── csp.py              # CSP tabanlı atama
│   ├── energy_model.py     # Fiziksel enerji hesaplama modeli
│   ├── ga.py               # Genetik Algoritma + 2-opt
│   └── data_generator.py   # Rastgele veri üreticisi
├── run_scenarios.py        # Senaryo test betiği
├── app.py                  # Streamlit arayüzü
├── server.py               # FastAPI HTTP & WS servisi
└── README.md               # Proje tanıtımı
```

## Geliştirme & Katkılar

- Farklı enerji modelleri (irmak, rüzgar haritaları) ekleyin.
- 3-opt veya Tabu Search gibi ek local search metodları entegre edin.
- Dockerfile ve CI/CD pipeline (GitHub Actions) oluşturun.
- Front-end kısmını React/Vue ile geliştirip WebSocket üzerinden canlı rota animasyonu yapın.

## Lisans

Bu proje MIT Lisansı ile lisanslanmıştır.
