/**
 * iotda.js — 调用华为云 IoTDA REST API，查询设备影子（最新属性）
 * 使用 IAM Token 认证（用户名+密码 → Token → API），无需 AK/SK 签名
 */
const axios = require('axios');
const config = require('../config');

// Token 缓存
let cachedToken = null;
let tokenExpiresAt = 0;  // 过期时间戳（毫秒）

/**
 * 获取 IAM Token
 * POST https://iam.cn-north-4.myhuaweicloud.com/v3/auth/tokens
 */
async function getToken() {
  // 如果 Token 还未过期（留 5 分钟余量），直接返回缓存
  if (cachedToken && Date.now() < tokenExpiresAt - config.tokenRefreshMargin * 1000) {
    return cachedToken;
  }

  const url = `https://${config.iamHost}/v3/auth/tokens`;

  const payload = {
    auth: {
      identity: {
        methods: ['password'],
        password: {
          user: {
            domain: { name: config.iamDomain },
            name: config.iamName,
            password: config.iamPassword
          }
        }
      },
      scope: {
        domain: { name: config.iamDomain }
      }
    }
  };

  console.log('[IAM] 获取 Token...');

  try {
    const res = await axios.post(url, payload, {
      headers: { 'Content-Type': 'application/json' },
      timeout: 10000
    });

    // Token 在响应头 X-Subject-Token 中
    const token = res.headers['x-subject-token'];
    if (!token) {
      console.error('[IAM] 响应头中未找到 X-Subject-Token');
      return null;
    }

    // Token 有效期通常为 24 小时，保守按 20 小时处理
    const ttl = 20 * 60 * 60 * 1000;
    cachedToken = token;
    tokenExpiresAt = Date.now() + ttl;

    console.log('[IAM] ✅ Token 获取成功，有效期约 20h');
    return token;
  } catch (error) {
    console.error('[IAM] Token 获取失败:', error.response?.status, error.message);
    if (error.response?.data) {
      console.error('[IAM]', JSON.stringify(error.response.data).substring(0, 500));
    }
    return null;
  }
}

/**
 * 查询设备影子（Device Shadow）
 * 使用应用接入端点（iotda-app），无需 Instance-Id
 * GET /v5/iot/{project_id}/devices/{device_id}/shadow
 */
async function queryDeviceShadow() {
  const token = await getToken();
  if (!token) {
    console.error('[IoTDA] 无有效 Token，跳过查询');
    return null;
  }

  const host = config.iotdaAppEndpoint;
  const uri = `/v5/iot/${config.projectId}/devices/${config.deviceId}/shadow`;
  const url = `https://${host}${uri}`;

  try {
    const response = await axios.get(url, {
      headers: {
        'X-Auth-Token': token
      },
      timeout: 5000
    });

    return response.data;
  } catch (error) {
    console.error('[IoTDA] 请求失败:', error.response?.status, error.message);
    if (error.response?.data) {
      console.error('[IoTDA]', JSON.stringify(error.response.data).substring(0, 500));
    }
    return null;
  }
}

/**
 * 解析设备影子，提取扁平化数据
 * 华为云影子格式：
 * {
 *   "device_id": "...",
 *   "shadow": [
 *     {
 *       "service_id": "ExerciseData",
 *       "reported": {
 *         "properties": { ... 设备上报的属性 ... }
 *       }
 *     }
 *   ]
 * }
 */
function parseShadow(shadowData) {
  if (!shadowData || !shadowData.shadow || shadowData.shadow.length === 0) {
    return null;
  }

  const props = {};

  // 遍历所有 service，合并 properties
  for (const svc of shadowData.shadow) {
    const reported = svc.reported?.properties;
    if (reported) {
      Object.assign(props, reported);
    }
  }

  return props;
}

module.exports = { queryDeviceShadow, parseShadow };
