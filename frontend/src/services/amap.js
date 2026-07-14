const AMAP_SCRIPT_ID = 'amap-js-api';

let loaderPromise;

function loadAmap() {
  if (window.AMap) {
    return Promise.resolve(window.AMap);
  }

  if (loaderPromise) {
    return loaderPromise;
  }

  const key = import.meta.env.VITE_AMAP_JS_KEY;
  if (!key) {
    return Promise.reject(
      new Error('未配置 VITE_AMAP_JS_KEY，位置搜索功能不可用')
    );
  }

  const serviceHost = import.meta.env.VITE_AMAP_SERVICE_HOST;
  const securityJsCode = import.meta.env.VITE_AMAP_SECURITY_CODE;
  if (serviceHost) {
    window._AMapSecurityConfig = {serviceHost};
  } else if (securityJsCode) {
    window._AMapSecurityConfig = {securityJsCode};
  }

  loaderPromise = new Promise((resolve, reject) => {
    const existingScript = document.getElementById(AMAP_SCRIPT_ID);
    if (existingScript) {
      existingScript.addEventListener('load', () => resolve(window.AMap), {once: true});
      existingScript.addEventListener('error', () => reject(new Error('高德地图 SDK 加载失败')), {once: true});
      return;
    }

    const script = document.createElement('script');
    script.id = AMAP_SCRIPT_ID;
    script.src = `https://webapi.amap.com/maps?v=2.0&key=${encodeURIComponent(key)}`;
    script.async = true;
    script.onload = () => resolve(window.AMap);
    script.onerror = () => {
      loaderPromise = undefined;
      reject(new Error('高德地图 SDK 加载失败'));
    };
    document.head.appendChild(script);
  });

  return loaderPromise;
}

function normalizeLocation(location) {
  if (!location) {
    return null;
  }

  if (typeof location.getLng === 'function' && typeof location.getLat === 'function') {
    return `${location.getLng()},${location.getLat()}`;
  }

  if (typeof location.lng === 'number' && typeof location.lat === 'number') {
    return `${location.lng},${location.lat}`;
  }

  return typeof location === 'string' ? location : null;
}

export async function searchAmapSuggestions(keyword) {
  const AMap = await loadAmap();

  return new Promise((resolve, reject) => {
    AMap.plugin('AMap.AutoComplete', () => {
      const autoComplete = new AMap.AutoComplete({
        city: '鹰潭',
        citylimit: true
      });

      autoComplete.search(keyword, (status, result) => {
        if (status !== 'complete' || !Array.isArray(result.tips)) {
          reject(new Error(result?.info || '高德位置建议查询失败'));
          return;
        }

        resolve(result.tips
          .map((tip, index) => ({
            id: tip.id || `${tip.name}-${index}`,
            name: tip.name,
            location: normalizeLocation(tip.location)
          }))
          .filter((tip) => tip.name && tip.location));
      });
    });
  });
}
