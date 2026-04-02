import { useLayoutEffect, useState } from 'react';

import {
  STACKED_LAYOUT_GAP_PX,
  STACKED_LAYOUT_MAX_WIDTH_PX,
  STACKED_LAYOUT_MIN_CHART_ROW_HEIGHT_PX,
  STACKED_LAYOUT_MIN_MAP_HEIGHT_PX,
  type DashboardLayout,
} from '../lib/appTypes';

export function useDashboardLayout(hostRef: React.RefObject<HTMLElement | null>) {
  const [dashboardLayout, setDashboardLayout] = useState<DashboardLayout>('split');

  useLayoutEffect(() => {
    const host = hostRef.current;
    if (!host) {
      return;
    }

    const updateLayout = () => {
      const width = host.clientWidth;
      const height = host.clientHeight;
      const canStack =
        width <= STACKED_LAYOUT_MAX_WIDTH_PX &&
        height >= STACKED_LAYOUT_MIN_MAP_HEIGHT_PX + STACKED_LAYOUT_MIN_CHART_ROW_HEIGHT_PX + STACKED_LAYOUT_GAP_PX;

      setDashboardLayout(canStack ? 'stacked' : 'split');
    };

    updateLayout();

    const resizeObserver = new ResizeObserver(updateLayout);
    resizeObserver.observe(host);
    return () => resizeObserver.disconnect();
  }, [hostRef]);

  return dashboardLayout;
}
