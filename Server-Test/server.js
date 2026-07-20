'use strict';

/**
 * IoT Node Test Server
 * ---------------------
 * Server pengujian untuk firmware ESP32-STM32F411-USART (+ add-on GSM
 * SIMCom A7670). Mengimplementasikan 3 endpoint yang wire-formatnya
 * PERSIS mengikuti kode firmware, supaya server ini bisa dipakai untuk
 * menguji jalur WiFi, Ethernet, MAUPUN GSM tanpa perbedaan apa pun di
 * sisi server - device yang menentukan lewat interface mana ia connect,
 * bukan server.
 *
 *   1. TCP        -> tcpSendJSONWithSeq() (WiFi/Eth) & gsmTcpSendJSONWithSeq() (GSM)
 *   2. HTTP POST   -> httpPostJSONWithSeq() (WiFi/Eth) & gsmHttpSendJSONWithSeq() (GSM)
 *   3. WebSocket   -> wsSendJSONWithSeq() (WiFi ONLY - tidak didukung Ethernet/GSM)
 *
 * Format ACK WAJIB persis seperti ini (tanpa spasi), karena firmware
 * mem-parsingnya pakai sscanf yang strict:
 *   {"ack_seq":<seq>,"status":"ok"}
 *
 * sscanf pattern di firmware (parseAck()):
 *   "{\"ack_seq\":%lu,\"status\":\"%7[^\"]\"}"
 */

const net = require('net');
const http = require('http');
const WebSocket = require('ws');

// ============ KONFIGURASI (bisa dioverride lewat environment variable) ============
const TCP_PORT  = parseInt(process.env.TCP_PORT  || '5001', 10);
const HTTP_PORT = parseInt(process.env.HTTP_PORT || '5002', 10);
const WS_PORT   = parseInt(process.env.WS_PORT   || '5003', 10);

// Path ini sekadar informasi log - HTTP & WS server di bawah menerima
// semua path (tidak divalidasi ketat) supaya gampang dipakai untuk
// testing tanpa harus config.json persis sama persen. Kalau mau lebih
// ketat, tinggal cocokkan req.url terhadap nilai ini.
const HTTP_PATH = process.env.HTTP_PATH || '/wsiot';
const WS_PATH   = process.env.WS_PATH   || '/wsiot';

// ============ HELPER ============
function logPayload(tag, raw) {
  const ts = new Date().toISOString();
  console.log(`\n[${tag}] ${ts}`);
  try {
    const obj = JSON.parse(raw);
    console.log(JSON.stringify(obj, null, 2));
  } catch (e) {
    console.log('  (JSON tidak valid, raw):', raw);
  }
}

function buildAck(seq) {
  return `{"ack_seq":${seq},"status":"ok"}`;
}

function extractSeq(raw) {
  try {
    const obj = JSON.parse(raw);
    return (typeof obj.seq === 'number') ? obj.seq : null;
  } catch (e) {
    return null;
  }
}

// =====================================================================
// 1. TCP SERVER
// Wire format: client kirim "<len desimal>\n<json persis len byte>",
// server balas "<ack json>\n" lalu koneksi ditutup (device selalu
// 1 pesan per koneksi: connect -> send -> tunggu ack -> stop()).
// =====================================================================
const tcpServer = net.createServer((socket) => {
  const addr = `${socket.remoteAddress}:${socket.remotePort}`;
  console.log(`[TCP] client connected: ${addr}`);

  let buf = Buffer.alloc(0);
  let expectedLen = null;

  socket.on('data', (chunk) => {
    buf = Buffer.concat([buf, chunk]);

    if (expectedLen === null) {
      const nlIndex = buf.indexOf('\n');
      if (nlIndex === -1) return; // header panjang belum lengkap
      const lenStr = buf.slice(0, nlIndex).toString('utf8').trim();
      buf = buf.slice(nlIndex + 1);
      expectedLen = parseInt(lenStr, 10);
      if (isNaN(expectedLen) || expectedLen < 0) {
        console.log(`[TCP] header panjang tidak valid: "${lenStr}"`);
        socket.end();
        return;
      }
    }

    if (buf.length >= expectedLen) {
      const jsonStr = buf.slice(0, expectedLen).toString('utf8');
      logPayload('TCP', jsonStr);

      const seq = extractSeq(jsonStr);
      if (seq !== null) {
        socket.write(buildAck(seq) + '\n');
        console.log(`[TCP] ACK terkirim (seq=${seq})`);
      } else {
        console.log('[TCP] field "seq" tidak ditemukan, ACK TIDAK dikirim');
      }
      socket.end();
    }
  });

  socket.on('error', (err) => console.log('[TCP] error:', err.message));
  socket.on('close', () => console.log(`[TCP] client disconnected: ${addr}`));
});

tcpServer.listen(TCP_PORT, () => {
  console.log(`[TCP]  listening on port ${TCP_PORT}`);
});

// =====================================================================
// 2. HTTP POST SERVER
// Body request = JSON persis, body respons = ACK JSON. Menerima semua
// path (tidak divalidasi ketat).
// =====================================================================
const httpServer = http.createServer((req, res) => {
  if (req.method !== 'POST') {
    res.writeHead(405, { 'Content-Type': 'text/plain' });
    res.end('Method Not Allowed - server ini hanya menerima POST');
    return;
  }

  let body = '';
  req.on('data', (chunk) => { body += chunk; });
  req.on('end', () => {
    logPayload('HTTP', body);

    const seq = extractSeq(body);
    if (seq === null) {
      console.log('[HTTP] field "seq" tidak ditemukan, ACK TIDAK dikirim');
      const errBody = '{"error":"invalid payload"}';
      res.writeHead(400, {
        'Content-Type': 'application/json',
        'Content-Length': Buffer.byteLength(errBody),
      });
      res.end(errBody);
      return;
    }

    const ack = buildAck(seq);
    res.writeHead(200, {
      'Content-Type': 'application/json',
      'Content-Length': Buffer.byteLength(ack),
    });
    res.end(ack);
    console.log(`[HTTP] ACK terkirim (seq=${seq})`);
  });

  req.on('error', (err) => console.log('[HTTP] error:', err.message));
});

httpServer.listen(HTTP_PORT, () => {
  console.log(`[HTTP] listening on port ${HTTP_PORT} (path referensi: ${HTTP_PATH}, semua path diterima)`);
});

// =====================================================================
// 3. WEBSOCKET SERVER (WiFi ONLY - sesuai firmware, Ethernet & GSM
// tidak mendukung transport ws)
// 1 text frame = 1 JSON, respons juga 1 text frame berisi ACK JSON.
// =====================================================================
const wsServer = new WebSocket.Server({ port: WS_PORT });

wsServer.on('connection', (ws, req) => {
  console.log(`[WS]  client connected: ${req.socket.remoteAddress} (path: ${req.url})`);

  ws.on('message', (data) => {
    const jsonStr = data.toString('utf8');
    logPayload('WS', jsonStr);

    const seq = extractSeq(jsonStr);
    if (seq !== null) {
      ws.send(buildAck(seq));
      console.log(`[WS] ACK terkirim (seq=${seq})`);
    } else {
      console.log('[WS] field "seq" tidak ditemukan, ACK TIDAK dikirim');
    }
  });

  ws.on('close', () => console.log('[WS] client disconnected'));
  ws.on('error', (err) => console.log('[WS] error:', err.message));
});

wsServer.on('listening', () => {
  console.log(`[WS]   listening on port ${WS_PORT} (path referensi: ${WS_PATH}, semua path diterima)`);
});

// =====================================================================
console.log('\n=== IoT Node Test Server siap ===');
console.log(`TCP  : port ${TCP_PORT}`);
console.log(`HTTP : port ${HTTP_PORT}   POST path: ${HTTP_PATH}`);
console.log(`WS   : port ${WS_PORT}   path: ${WS_PATH}`);
console.log('\nSamakan config.json ESP32 -> "transport.host" = IP mesin ini,');
console.log('"transport.port" = salah satu port di atas sesuai "transport.type"\n');
