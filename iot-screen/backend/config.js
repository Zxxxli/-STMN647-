/**
 * config.js — 集中管理所有配置，从 .env 加载
 */
require('dotenv').config();

module.exports = {
  // IAM 认证
  iamName: process.env.HW_IAM_NAME,
  iamPassword: process.env.HW_IAM_PASSWORD,
  iamDomain: process.env.HW_IAM_DOMAIN,

  projectId: process.env.HW_PROJECT_ID,
  region: process.env.HW_REGION,
  deviceId: process.env.DEVICE_ID,
  port: process.env.PORT || 3001,

  // IoTDA 实例 ID
  instanceId: process.env.HW_IOTDA_INSTANCE_ID,

  // IoTDA 应用接入地址（实例专属）
  iotdaAppEndpoint: process.env.HW_IOTDA_APP_ENDPOINT,

  // IoTDA REST API 主机地址
  iotdaHost: 'iotda.cn-north-4.myhuaweicloud.com',

  // IAM 端点
  iamHost: 'iam.cn-north-4.myhuaweicloud.com',

  // 轮询间隔（毫秒）
  pollInterval: 500,

  // Token 提前刷新时间（秒）
  tokenRefreshMargin: 300
};
