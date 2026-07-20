import { useMemo } from 'react';
import * as THREE from 'three';
import type { Vec2 } from '../types';

interface NFZPrismProps {
  boundaryPoints: Vec2[];
  altMin: number;
  altMax: number;
}

export function NFZPrism({ boundaryPoints, altMin, altMax }: NFZPrismProps) {
  const shape = useMemo(() => {
    const s = new THREE.Shape();
    if (boundaryPoints.length === 0) return s;
    s.moveTo(boundaryPoints[0][0], boundaryPoints[0][1]);
    for (let i = 1; i < boundaryPoints.length; i++) {
      s.lineTo(boundaryPoints[i][0], boundaryPoints[i][1]);
    }
    s.closePath();
    return s;
  }, [boundaryPoints]);

  const height = altMax - altMin;
  if (height <= 0 || boundaryPoints.length < 3) return null;

  return (
    <mesh position={[0, altMin, 0]} rotation={[-Math.PI / 2, 0, 0]}>
      <extrudeGeometry
        args={[shape, { steps: 1, depth: height, bevelEnabled: false }]}
      />
      <meshBasicMaterial
        color="#ff8800"
        transparent
        opacity={0.25}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}
