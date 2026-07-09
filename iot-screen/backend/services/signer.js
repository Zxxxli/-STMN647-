/**
 * signer.js — 华为云 AK/SK 签名（SDK-HMAC-SHA256）
 * 用于对 IoTDA REST API 请求进行身份认证
 *
 * 关键：华为云 SDK-HMAC-SHA256 直接用 SK 对 stringToSign 做一次 HMAC，
 * 不是 AWS V4 的多级密钥派生！这是与 AWS 签名最大的区别。
 *
 * 参考华为云官方 SDK：
 * @huaweicloud/huaweicloud-sdk-core/auth/AKSKSigner.js
 */
const crypto = require('crypto');

const EMPTY_BODY_SHA256 = 'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855';

// URL 编码所需的查找表（与官方 SDK 一致）
const HEX_TABLE = [];
for (let i = 0; i < 256; ++i) {
  HEX_TABLE[i] = '%' + ((i < 16 ? '0' : '') + i.toString(16)).toUpperCase();
}

// 不需要转义的 ASCII 字符（与官方 SDK 一致）
const NO_ESCAPE = [
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1,
  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0
];

/**
 * URL 编码（与官方 SDK urlEncode 一致）
 * 只对特殊字符编码，保留不编码的 ASCII 字符
 */
function urlEncode(str) {
  if (typeof str !== 'string') {
    str = String(str);
  }
  let out = '';
  let lastPos = 0;
  for (let i = 0; i < str.length; ++i) {
    let c = str.charCodeAt(i);
    if (c < 0x80) {
      if (NO_ESCAPE[c] === 1) continue;
      if (lastPos < i) out += str.slice(lastPos, i);
      lastPos = i + 1;
      out += HEX_TABLE[c];
      continue;
    }
    if (lastPos < i) out += str.slice(lastPos, i);
    if (c < 0x800) {
      lastPos = i + 1;
      out += HEX_TABLE[0xC0 | (c >> 6)] + HEX_TABLE[0x80 | (c & 0x3F)];
      continue;
    }
    if (c < 0xD800 || c >= 0xE000) {
      lastPos = i + 1;
      out += HEX_TABLE[0xE0 | (c >> 12)] +
             HEX_TABLE[0x80 | ((c >> 6) & 0x3F)] +
             HEX_TABLE[0x80 | (c & 0x3F)];
      continue;
    }
    ++i;
    if (i >= str.length) throw new Error('ERR_INVALID_URI');
    const c2 = str.charCodeAt(i) & 0x3FF;
    lastPos = i + 1;
    c = 0x10000 + (((c & 0x3FF) << 10) | c2);
    out += HEX_TABLE[0xF0 | (c >> 18)] +
           HEX_TABLE[0x80 | ((c >> 12) & 0x3F)] +
           HEX_TABLE[0x80 | ((c >> 6) & 0x3F)] +
           HEX_TABLE[0x80 | (c & 0x3F)];
  }
  if (lastPos === 0) return str;
  if (lastPos < str.length) return out + str.slice(lastPos);
  return out;
}

/**
 * 构建规范 URI：每个路径段 URL 编码，末尾加 /
 */
function canonicalURI(uri) {
  if (!uri) return uri;
  const segments = uri.split('/');
  const encoded = segments.map(s => urlEncode(s));
  let path = encoded.join('/');
  if (path[path.length - 1] !== '/') {
    path = path + '/';
  }
  return path;
}

/**
 * SHA256 哈希，返回小写十六进制
 */
function sha256Hex(str) {
  return crypto.createHash('sha256').update(str).digest('hex');
}

/**
 * HMAC-SHA256，返回小写十六进制
 */
function hmacSHA256(key, str) {
  return crypto.createHmac('sha256', key).update(str).digest('hex');
}

class Signer {
  constructor(ak, sk, region, service) {
    this.ak = ak;
    this.sk = sk;
    this.region = region;
    this.service = service;
  }

  /**
   * 对一次 HTTP 请求生成签名头
   * @param {string}  method       - GET/POST/PUT/DELETE
   * @param {string}  host         - 主机名，如 iotda.cn-north-4.myhuaweicloud.com
   * @param {string}  uri          - 请求路径，如 /v5/iot/xxx/devices/xxx/shadow
   * @param {string}  query        - 查询参数（可选，含 ? 前缀），无则留空
   * @param {string}  payload      - 请求体，GET 请求为空字符串
   * @param {object}  extraHeaders - 额外需要签名的请求头（如 { 'Instance-Id': 'xxx' }）
   * @returns {object} 包含 X-Sdk-Date, Authorization, host 及所有 extraHeaders
   */
  sign(method, host, uri, query = '', payload = '', extraHeaders = {}) {
    // 生成 UTC 时间戳（格式：YYYYMMDDTHHMMSSZ）
    const now = new Date();
    const pad = (n, d = 2) => String(n).padStart(d, '0');
    const timestamp =
      now.getUTCFullYear().toString() +
      pad(now.getUTCMonth() + 1) +
      pad(now.getUTCDate()) +
      'T' +
      pad(now.getUTCHours()) +
      pad(now.getUTCMinutes()) +
      pad(now.getUTCSeconds()) +
      'Z';

    // 1. 构建规范 URI（URL 编码 + 末尾 /）
    const canonicalUri = canonicalURI(uri);

    // 2. 构建规范查询字符串
    const canonicalQuery = query;

    // 3. 构建规范 Headers — 合并必须头 + 额外头，全部小写排序后纳入签名
    const allHeaders = {
      'host': host,
      'x-sdk-date': timestamp,
      ...extraHeaders           // 如 Instance-Id 等业务头也参与签名
    };
    const sortedKeys = Object.keys(allHeaders).sort();
    const signedHeaderNames = sortedKeys.join(';');
    const canonicalHeaders = sortedKeys
      .map(k => `${k}:${allHeaders[k]}`)
      .join('\n') + '\n';

    // 4. 请求体哈希（GET 请求为空体的哈希）
    const hashedPayload = payload
      ? sha256Hex(payload)
      : EMPTY_BODY_SHA256;

    // 5. 构建规范请求
    const canonicalRequest = [
      method,
      canonicalUri,
      canonicalQuery,
      canonicalHeaders,
      signedHeaderNames,
      hashedPayload
    ].join('\n');

    // 6. 构建待签字符串
    const hashedCanonicalRequest = sha256Hex(canonicalRequest);
    const stringToSign = [
      'SDK-HMAC-SHA256',
      timestamp,
      hashedCanonicalRequest
    ].join('\n');

    // ★★★ 7. 关键区别：华为云直接用 SK 对 stringToSign 做 HMAC，
    // 不是 AWS V4 的 kDate→kRegion→kService→signingKey 多级派生！★★★
    const signature = hmacSHA256(this.sk, stringToSign);

    return {
      'X-Sdk-Date': timestamp,
      'Authorization': `SDK-HMAC-SHA256 Access=${this.ak}, SignedHeaders=${signedHeaderNames}, Signature=${signature}`,
      'host': host,
      ...extraHeaders            // 返回时也带上额外头
    };
  }
}

module.exports = Signer;
