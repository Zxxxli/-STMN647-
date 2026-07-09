/**
 * app.js — Node.js 中继服务入口
 *
 * 职责：
 *   1. 每 2 秒轮询华为云 IoTDA 设备影子
 *   2. 更新内存缓存
 *   3. 提供 /api/dashboard 接口供前端调用
 */
const express = require('express');
const path = require('path');
const cors = require('cors');
const config = require('./config');
const { queryDeviceShadow, parseShadow } = require('./services/iotda');
const { updateCache, markSuccess } = require('./cache/dashboardCache');
const dashboardRouter = require('./routes/dashboard');

const app = express();
app.use(cors());
app.use(express.json());

// 托管前端静态文件（index.html 在上级目录）
app.use(express.static(path.join(__dirname, '..')));

// 注册路由
app.use('/api', dashboardRouter);

// 健康检查
app.get('/health', (req, res) => res.json({ status: 'ok' }));

// ==========================================
// 定时轮询 IoTDA 设备影子
// ==========================================
async function pollDeviceData() {
  console.log('[轮询] 查询设备影子...');

  const shadow = await queryDeviceShadow();

  if (shadow) {
    const props = parseShadow(shadow);
    if (props) {
      markSuccess();
      updateCache(props);
      console.log('[轮询] ✅ 数据已更新, keys:', Object.keys(props).join(', '));
    } else {
      console.log('[轮询] ⚠️  影子为空，设备可能尚未上报');
      updateCache(null);
    }
  } else {
    console.log('[轮询] ❌ 查询失败，保持旧缓存');
    updateCache(null);
  }
}

// 启动时立即查一次
pollDeviceData();

// 之后定时轮询
setInterval(pollDeviceData, config.pollInterval);

// ==========================================
// 启动服务
// ==========================================
app.listen(config.port, () => {
  console.log(`\n🚀 数据中继服务已启动: http://localhost:${config.port}`);
  console.log(`📡 设备 ID: ${config.deviceId}`);
  console.log(`🔄 轮询间隔: ${config.pollInterval}ms\n`);
});
