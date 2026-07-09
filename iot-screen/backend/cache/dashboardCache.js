/**
 * dashboardCache.js — 内存缓存层
 * 保存最新一次设备数据，前端请求时直接返回缓存，不重复访问华为云
 */

// 缓存对象
let cache = {
  lastUpdate: null,       // 最后更新时间戳
  deviceOnline: false,    // 设备是否在线
  rawData: null           // 原始影子属性
};

/**
 * 更新缓存
 * @param {object|null} props - 从设备影子解析出的属性，null 表示查询失败
 */
function updateCache(props) {
  if (props) {
    cache.lastUpdate = new Date().toISOString();
    cache.deviceOnline = true;
    cache.rawData = props;
  } else {
    // 查询失败时保持设备在线状态不变，只更新 offline 标志
    // 连续失败 5 次才标记离线
    if (!cache._failCount) cache._failCount = 0;
    cache._failCount++;
    if (cache._failCount >= 5) {
      cache.deviceOnline = false;
    }
  }
}

/** 重置失败计数（成功后调用） */
function markSuccess() {
  cache._failCount = 0;
  cache.deviceOnline = true;
}

/**
 * 获取当前缓存（组装成前端需要的统一格式）
 */
function getCache() {
  const data = cache.rawData || {};

  return {
    timestamp: cache.lastUpdate,
    online: cache.deviceOnline,

    // 运动计数
    squat: data.squat ?? 0,    // 深蹲个数
    jumpe: data.jumpe ?? 0,    // 开合跳个数
    pbtim: data.pbtim ?? 0,    // 平板支撑计时 (秒)

    // 动作质量参数
    sqdee: data.sqdee ?? 0,    // 深蹲深度
    sqbac: data.sqbac ?? 0,    // 深蹲时背部评估 (0=标准, 1=异常)
    jopen: data.jopen ?? 0,    // 开合跳舒展度评估
    pbbod: data.pbbod ?? 0,    // 平板支撑姿态评估 (0=标准, 1=异常)

    // 综合评分（由质量参数计算，若无则默认80）
    score: data.score ?? 80
  };
}

module.exports = { cache, updateCache, markSuccess, getCache };
