'use strict';
// Client simulasi meniru PERSIS wire-format firmware, untuk verifikasi
// server. Bukan untuk dipakai user - cuma alat bantu QA sebelum diserahkan.

const net = require('net');
const http = require('http');
const WebSocket = require('ws');

const sample = JSON.stringify({
  timestamp: "2026-07-13T10:00:00Z",
  env: { temp: 25.3, hum: 60.1 },
  dig_in: [0,1,0,0,0,0,0,0,0,0,0,0],
  an_in: [111,222,333,444],
  q: [0,1,0,0],
  seq: 42
});

function testTcp() {
  return new Promise((resolve, reject) => {
    const socket = net.createConnection(5001, '127.0.0.1', () => {
      socket.write(String(sample.length) + '\n');
      socket.write(sample);
    });
    let resp = '';
    socket.on('data', (d) => { resp += d.toString(); });
    socket.on('close', () => {
      const ok = resp.trim() === '{"ack_seq":42,"status":"ok"}';
      console.log(`[TEST][TCP]  resp="${resp.trim()}" -> ${ok ? 'PASS' : 'FAIL'}`);
      ok ? resolve() : reject(new Error('TCP ack mismatch'));
    });
    socket.on('error', reject);
  });
}

function testHttp() {
  return new Promise((resolve, reject) => {
    const req = http.request({
      hostname: '127.0.0.1',
      port: 5002,
      path: '/wsiot',
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Content-Length': Buffer.byteLength(sample),
      },
    }, (res) => {
      let body = '';
      res.on('data', (c) => body += c);
      res.on('end', () => {
        const ok = body === '{"ack_seq":42,"status":"ok"}' && res.statusCode === 200;
        console.log(`[TEST][HTTP] status=${res.statusCode} body="${body}" -> ${ok ? 'PASS' : 'FAIL'}`);
        ok ? resolve() : reject(new Error('HTTP ack mismatch'));
      });
    });
    req.on('error', reject);
    req.write(sample);
    req.end();
  });
}

function testWs() {
  return new Promise((resolve, reject) => {
    const ws = new WebSocket('ws://127.0.0.1:5003/wsiot');
    ws.on('open', () => ws.send(sample));
    ws.on('message', (data) => {
      const resp = data.toString();
      const ok = resp === '{"ack_seq":42,"status":"ok"}';
      console.log(`[TEST][WS]   resp="${resp}" -> ${ok ? 'PASS' : 'FAIL'}`);
      ws.close();
      ok ? resolve() : reject(new Error('WS ack mismatch'));
    });
    ws.on('error', reject);
  });
}

(async () => {
  try {
    await testTcp();
    await testHttp();
    await testWs();
    console.log('\nALL TESTS PASSED');
    process.exit(0);
  } catch (e) {
    console.error('\nTEST FAILED:', e.message);
    process.exit(1);
  }
})();
