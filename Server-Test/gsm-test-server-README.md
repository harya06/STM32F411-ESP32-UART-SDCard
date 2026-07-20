# IoT Node Test Server

Server Node.js untuk menguji 3 transport firmware `ESP32-STM32F411-USART`
(+ add-on GSM): **TCP**, **HTTP POST**, dan **WebSocket** (WiFi only).
Server ini **tidak peduli** device connect lewat WiFi, Ethernet, atau GSM —
wire-format-nya sudah dibuat identik di sisi firmware untuk ketiga
interface itu, jadi 1 server ini cukup untuk uji ketiganya.

## Instalasi

```bash
npm install
```

## Menjalankan

```bash
npm start
# atau
node server.js
```

Default port:

| Transport | Port | Env var override |
|---|---|---|
| TCP | 5001 | `TCP_PORT` |
| HTTP POST | 5002 | `HTTP_PORT` |
| WebSocket | 5003 | `WS_PORT` |

Contoh ganti port:
```bash
TCP_PORT=6001 HTTP_PORT=6002 WS_PORT=6003 node server.js
```

## Konfigurasi `config.json` di device

Cari tahu dulu IP mesin yang menjalankan server ini:
```bash
# Linux/Mac
ip addr | grep inet
# atau
hostname -I
```

Lalu isi `transport.host` dengan IP tersebut, dan `transport.port` sesuai
tabel di atas mengikuti `transport.type` yang dipilih:

**TCP** (WiFi/Ethernet/GSM):
```json
"transport": { "type": "tcp", "host": "192.168.1.50", "port": 5001, "path": "/wsiot" }
```

**HTTP POST** (WiFi/Ethernet/GSM):
```json
"transport": { "type": "httppost", "host": "192.168.1.50", "port": 5002, "path": "/wsiot" }
```

**WebSocket** (WiFi ONLY - akan ditolak firmware kalau `network.mode`
selain `"wifi"`):
```json
"network": { "mode": "wifi" },
"transport": { "type": "ws", "host": "192.168.1.50", "port": 5003, "path": "/wsiot" }
```

## Testing tanpa hardware (opsional)

`test-client.js` meniru persis wire-format firmware (dipakai untuk
verifikasi server ini sebelum diserahkan) - bisa dipakai juga untuk
sanity-check cepat sebelum coba ke hardware asli:

```bash
node server.js &
node test-client.js
```

Kalau ketiganya `PASS`, server sudah siap dipakai untuk uji hardware.

## Catatan penting untuk pengujian GSM

Modem GSM terhubung lewat jaringan **seluler operator**, BUKAN LAN yang
sama dengan laptop/PC Anda. Supaya modem bisa mencapai server ini:

- **Untuk uji cepat di jaringan lokal**: tidak bisa langsung, karena data
  seluler biasanya di-NAT operator (device tidak bisa akses IP privat
  `192.168.x.x` laptop Anda dari luar).
- **Opsi 1 - Deploy ke server publik**: jalankan server ini di VPS/cloud
  (DigitalOcean, AWS, dst) yang punya IP publik, lalu isi `transport.host`
  dengan IP publik tersebut.
- **Opsi 2 - Tunnel** (untuk testing cepat tanpa deploy): pakai `ngrok`
  atau sejenisnya untuk TCP/HTTP:
  ```bash
  ngrok tcp 5001      # untuk transport TCP
  ngrok http 5002     # untuk transport HTTP POST
  ```
  lalu pakai host/port yang diberikan ngrok di `config.json`.
  (WebSocket tidak relevan untuk GSM karena memang tidak didukung.)

WiFi dan Ethernet tidak punya kendala ini selama laptop/PC dan
ESP32 berada di jaringan/LAN yang sama (atau device di-set ke IP publik
server kalau diakses dari luar LAN).

## Format data (referensi)

Payload yang dikirim device (semua transport, semua interface):
```json
{
  "timestamp": "2026-07-13T10:00:00Z",
  "env": { "temp": 25.3, "hum": 60.1 },
  "dig_in": [0,1,0,0,0,0,0,0,0,0,0,0],
  "an_in": [111,222,333,444],
  "q": [0,1,0,0],
  "seq": 42
}
```

ACK yang WAJIB dibalas server (format ketat, firmware parsing pakai
`sscanf`, tidak boleh ada spasi tambahan):
```
{"ack_seq":42,"status":"ok"}
```
