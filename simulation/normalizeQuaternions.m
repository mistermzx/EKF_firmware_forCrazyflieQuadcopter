function normalizedState = normalizeQuaternions(state)
    q0 = state(7);
    q1 = state(8);
    q2 = state(9);
    q3 = state(10);
    recipNorm = 1/sqrt(q0^2+q1^2+q2^2+q3^2);
    normalizedState = state;
    normalizedState(7) = q0*recipNorm;
    normalizedState(8) = q1*recipNorm;
    normalizedState(9) = q2*recipNorm;
    normalizedState(10) = q3*recipNorm;
end