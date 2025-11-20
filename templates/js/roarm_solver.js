export const roarm_m2 = (() => {
    const ARM_L1_LENGTH_MM = 126.06;
    const ARM_L2_LENGTH_MM_A = 236.82;
    const ARM_L2_LENGTH_MM_B = 30.00;
    const ARM_L3_LENGTH_MM_A_0 = 280.15;
    const ARM_L3_LENGTH_MM_B_0 = 1.73;
    const ARM_L4_LENGTH_MM_A = 67.85;
    const ARM_L4_LENGTH_MM_B = 5.98;

    const l1 = ARM_L1_LENGTH_MM;
    const l2A = ARM_L2_LENGTH_MM_A;
    const l2B = ARM_L2_LENGTH_MM_B;
    const l2 = Math.sqrt(l2A * l2A + l2B * l2B);
    const t2rad = Math.atan2(l2B, l2A);

    const l3A = ARM_L3_LENGTH_MM_A_0;
    const l3B = ARM_L3_LENGTH_MM_B_0;
    const l3 = Math.sqrt(l3A * l3A + l3B * l3B);
    const t3rad = Math.atan2(l3B, l3A);

    const l4A = ARM_L4_LENGTH_MM_A;
    const l4B = ARM_L4_LENGTH_MM_B;

    let EOAT_point_RAD_BUFFER = 0;
    let nanIK = false;
    let EEMode = 0; 

    let EoAT_A = 0;
    let EoAT_B = 0;
    let lEA = EoAT_A + l4A;
    let lEB = EoAT_B + l4B;
    let lE = Math.sqrt(lEA * lEA + lEB * lEB);
    let tErad = Math.atan2(lEB, lEA);

    let lastX = l3A + l2B;
    let lastY = 0;
    let lastZ = l2A - l3B;
    let lastT = Math.PI;

    function polarToCartesian(r, theta) {
        return [r * Math.cos(theta), r * Math.sin(theta)];
    }

    function cartesianToPolar(x, y) {
        return [Math.sqrt(x * x + y * y), Math.atan2(y, x)];
    }

    function simpleLinkageIkRad(aIn, bIn) {
        let LA = l2;
        let LB = l3;
        let psi, alpha, omega, beta, L2C, LC, lambda;

        if (Math.abs(bIn) < 1e-6) {
            psi = Math.acos((LA * LA + aIn * aIn - LB * LB) / (2 * LA * aIn)) + t2rad;
            alpha = Math.PI / 2 - psi;
            omega = Math.acos((aIn * aIn + LB * LB - LA * LA) / (2 * aIn * LB));
            beta = psi + omega - t3rad;
        } else {
            L2C = aIn * aIn + bIn * bIn;
            LC = Math.sqrt(L2C);
            lambda = Math.atan2(bIn, aIn);
            psi = Math.acos((LA * LA + L2C - LB * LB) / (2 * LA * LC)) + t2rad;
            alpha = Math.PI / 2 - lambda - psi;
            omega = Math.acos((LB * LB + L2C - LA * LA) / (2 * LC * LB));
            beta = psi + omega - t3rad;
        }

        let delta = Math.PI / 2 - alpha - beta;
        EOAT_point_RAD_BUFFER = delta;
        nanIK = isNaN(alpha) || isNaN(beta) || isNaN(delta);

        return [alpha, beta];

    }

    function computePosbyJointRad(baseRad, shoulderRad, elbowRad, handRad) {
        let x_ee, y_ee, z_ee, r_ee;

        if (EEMode === 0) {
            let [aOut, bOut] = polarToCartesian(l2, Math.PI / 2 - (shoulderRad + t2rad));
            let [cOut, dOut] = polarToCartesian(l3, Math.PI / 2 - (elbowRad + shoulderRad + t3rad));
            r_ee = aOut + cOut;
            z_ee = bOut + dOut;
            [x_ee, y_ee] = polarToCartesian(r_ee, baseRad);
            lastX = x_ee;
            lastY = y_ee;
            lastZ = z_ee;
        } else if (EEMode === 1) {
            let [aOut, bOut] = polarToCartesian(l2, Math.PI / 2 - (shoulderRad + t2rad));
            let [cOut, dOut] = polarToCartesian(l3, Math.PI / 2 - (elbowRad + shoulderRad + t3rad));
            let [eOut, fOut] = polarToCartesian(lE, -((handRad + tErad) - Math.PI - (Math.PI / 2 - shoulderRad - elbowRad)));
            r_ee = aOut + cOut + eOut;
            z_ee = bOut + dOut + fOut;
            [x_ee, y_ee] = polarToCartesian(r_ee, baseRad);
            lastX = x_ee;
            lastY = y_ee;
            lastZ = z_ee;
            lastT = handRad - (Math.PI - shoulderRad - elbowRad) + Math.PI / 2;
        }

        return [lastX, lastY, lastZ, EEMode, lastT];
    }

    function computeJointRadbyPos(x, y, z, t) {
        const [r, theta] = cartesianToPolar(x, y);
        const [shoulder, elbow] = simpleLinkageIkRad(r, z);
        return [theta, shoulder, elbow];
    }

    return {
        polarToCartesian,
        cartesianToPolar,
        simpleLinkageIkRad,
        computePosbyJointRad,
        computeJointRadbyPos,
        setEEMode: mode => EEMode = mode,
        getNanIK: () => nanIK,
    };
})();

export const roarm_m3 = (() => {
  const ARM_L1_LENGTH_MM = 126.06;
  const ARM_L2_LENGTH_MM_A = 236.82;
  const ARM_L2_LENGTH_MM_B = 30.0;
  const ARM_L3_LENGTH_MM_A_0 = 144.49;
  const ARM_L3_LENGTH_MM_B_0 = 0;
  const ARM_L3_LENGTH_MM_A_1 = 144.49;
  const ARM_L3_LENGTH_MM_B_1 = 0;
  const ARM_L4_LENGTH_MM_A = 171.67;
  const ARM_L4_LENGTH_MM_B = 13.69;

  const l1 = ARM_L1_LENGTH_MM;
  const l2A = ARM_L2_LENGTH_MM_A;
  const l2B = ARM_L2_LENGTH_MM_B;
  const l2 = Math.sqrt(l2A * l2A + l2B * l2B);
  const t2rad = Math.atan2(l2B, l2A);
  const l3A = ARM_L3_LENGTH_MM_A_0;
  const l3B = ARM_L3_LENGTH_MM_B_0;
  const l3 = Math.sqrt(l3A * l3A + l3B * l3B);
  const t3rad = Math.atan2(l3B, l3A);
  const EoAT_A = 0;
  const EoAT_B = 0;
  const l4A = ARM_L4_LENGTH_MM_A;
  const l4B = ARM_L4_LENGTH_MM_B;
  const lEA = EoAT_A + ARM_L4_LENGTH_MM_A;
  const lEB = EoAT_B + ARM_L4_LENGTH_MM_B;
  const lE = Math.sqrt(lEA * lEA + lEB * lEB);
  const tErad = Math.atan2(lEB, lEA);

  let SHOULDER_JOINT_RAD = 0;
  let ELBOW_JOINT_RAD = Math.PI / 2;
  let EOAT_JOINT_RAD_BUFFER;

  let nanIK = false;
  let nanFK = false;

  let lastX = l2B + l3A + ARM_L4_LENGTH_MM_A;
  let lastY = 0;
  let lastZ = l2A - ARM_L4_LENGTH_MM_B;
  let lastT = 0;
  let lastR = 0;
  let lastG = 3.14;

  let lastValidResult = [0, 0, 0, 0, 0];

  function simpleLinkageIkRad(aIn, bIn) {
    let psi, alpha, omega, beta;
    let L2C, LC, lambda, delta;
    const LA = l2;
    const LB = l3;

    if (Math.abs(bIn) < 1e-6) {
      psi = Math.acos((LA * LA + aIn * aIn - LB * LB) / (2 * LA * aIn)) + t2rad;
      alpha = Math.PI / 2 - psi;
      omega = Math.acos((aIn * aIn + LB * LB - LA * LA) / (2 * aIn * LB));
      beta = psi + omega - t3rad;
    } else {
      L2C = aIn * aIn + bIn * bIn;
      LC = Math.sqrt(L2C);
      lambda = Math.atan2(bIn, aIn);
      psi = Math.acos((LA * LA + L2C - LB * LB) / (2 * LA * LC)) + t2rad;
      alpha = Math.PI / 2 - lambda - psi;
      omega = Math.acos((LB * LB + L2C - LA * LA) / (2 * LC * LB));
      beta = psi + omega - t3rad;
    }

    delta = Math.PI / 2 - alpha - beta;

    SHOULDER_JOINT_RAD = alpha;
    ELBOW_JOINT_RAD = beta;
    EOAT_JOINT_RAD_BUFFER = delta;

    nanIK = Number.isNaN(alpha) || Number.isNaN(beta) || Number.isNaN(delta);
    return [SHOULDER_JOINT_RAD, ELBOW_JOINT_RAD, EOAT_JOINT_RAD_BUFFER];
  }

  function rotatePoint(theta) {
    const alpha = tErad + theta;
    const xB = -lE * Math.cos(alpha);
    const yB = -lE * Math.sin(alpha);
    return [xB, yB];
  }

  function movePoint(xA, yA, s) {
    const distance = Math.sqrt(xA * xA + yA * yA);
    let xB, yB;
    if (distance - s <= 1e-6) {
      xB = 0;
      yB = 0;
    } else {
      const ratio = (distance - s) / distance;
      xB = xA * ratio;
      yB = yA * ratio;
    }
    return [xB, yB];
  }

  function cartesianToPolar(x, y) {
    const r = Math.sqrt(x * x + y * y);
    const theta = Math.atan2(y, x);
    return [r, theta];
  }

  function polarToCartesian(r, theta) {
    const x = r * Math.cos(theta);
    const y = r * Math.sin(theta);
    return [x, y];
  }

  function computePosbyJointRad(base_joint_rad, shoulder_joint_rad, elbow_joint_rad, wrist_joint_rad, roll_joint_rad, hand_joint_rad) {
    const [aOut, bOut] = polarToCartesian(l2, Math.PI / 2 - (shoulder_joint_rad + t2rad));
    const [cOut, dOut] = polarToCartesian(l3, Math.PI / 2 - (elbow_joint_rad + shoulder_joint_rad + t3rad));
    const [eOut, fOut] = polarToCartesian(lE, Math.PI / 2 - (elbow_joint_rad + shoulder_joint_rad + wrist_joint_rad + tErad));

    const r_ee = aOut + cOut + eOut;
    const z_ee = bOut + dOut + fOut;

    const [gOut, hOut] = polarToCartesian(r_ee, base_joint_rad);

    lastX = gOut;
    lastY = hOut;
    lastZ = z_ee;
    lastT = elbow_joint_rad + shoulder_joint_rad + wrist_joint_rad - Math.PI / 2;

    return [lastX, lastY, lastZ, roll_joint_rad, lastT];
  }

  function computeJointRadbyPos(x, y, z, roll, pitch, hand_joint_rad) {

    const delta = rotatePoint(pitch - 3.1416);
    const beta = movePoint(x, y, delta[0]);
    const bases = cartesianToPolar(beta[0], beta[1]);
    const radians = simpleLinkageIkRad(bases[0], z + delta[1]);

    const WRIST_JOINT_RAD = radians[2] + pitch;
    const ROLL_JOINT_RAD = roll;

    const result = [bases[1], radians[0], radians[1], WRIST_JOINT_RAD, ROLL_JOINT_RAD, hand_joint_rad];

    if (result.every(v => !Number.isNaN(v))) {
      lastValidResult = result;
      return result;
    } else {
      console.warn("Inverse kinematics returned NaN. Using last valid result.");
      return lastValidResult;
    }
  }

  return {
    simpleLinkageIkRad,
    rotatePoint,
    movePoint,
    cartesianToPolar,
    polarToCartesian,
    computePosbyJointRad,
    computeJointRadbyPos,
    getNanIK: () => nanIK,
  };
})();
