import { useMemo } from 'react';
import type { Vec3 } from '../types';

function toThreePos([x, y, z]: Vec3): [number, number, number] {
  return [x, z, y];
}

interface TargetZoneProps {
  center: Vec3;
  radius: number;
}

export function TargetZone({ center, radius }: TargetZoneProps) {
  const pos = useMemo(() => toThreePos(center), [center]);

  return (
    <mesh position={pos}>
      <sphereGeometry args={[radius, 48, 24]} />
      <meshBasicMaterial color="#4488ff" transparent opacity={0.25} />
    </mesh>
  );
}
