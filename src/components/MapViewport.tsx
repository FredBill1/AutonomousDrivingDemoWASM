import type { MapViewportProps } from './mapViewportTypes';
import { useMapViewportRuntime } from './useMapViewportRuntime';

export function MapViewport(props: MapViewportProps) {
  const hostRef = useMapViewportRuntime(props);
  return <div ref={hostRef} className="map-view" />;
}
