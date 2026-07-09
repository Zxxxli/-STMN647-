/**
 * dashboard.js — REST API 路由
 * GET /api/dashboard → 返回最新设备数据（来自缓存）
 */
const express = require('express');
const router = express.Router();
const { getCache } = require('../cache/dashboardCache');

router.get('/dashboard', (req, res) => {
  const data = getCache();
  res.json(data);
});

module.exports = router;
