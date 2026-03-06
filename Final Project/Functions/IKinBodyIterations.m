function [thetalist, success, theta_mat] = IKinBodyIterations(Blist, M, T, thetalist0, eomg, ev, csv_filename)
% IKinBodyIterations
% Same as IKinBody, but prints an iteration report (starting at i=0)
% and saves the joint vector at each iteration as a row in a CSV file.
%
% Inputs/outputs match IKinBody, plus:
%   csv_filename (optional): output CSV name (default: 'IKinBodyIterations.csv')

% Returns:
%   thetalist  : final joint angles (nx1)
%   success    : true if converged within tolerances
%   theta_mat  : (k x n) matrix of joint angles per iteration (row = iteration)

    if nargin < 7 || isempty(csv_filename)
        csv_filename = 'IKinBodyIterations.csv';
    end

    thetalist = thetalist0;
    i = 0;
    maxiterations = 20;

    % Preallocate (maxiterations+1 rows, n cols). We'll trim at the end.
    n = length(thetalist0);
    theta_mat = zeros(maxiterations + 1, n);

    % --- Iteration 0 (initial guess) ---
    Tsb = FKinBody(M, Blist, thetalist);
    Vb  = se3ToVec(MatrixLog6(TransInv(Tsb) * T));
    omg_err = norm(Vb(1:3));
    v_err   = norm(Vb(4:6));
    err = (omg_err > eomg) || (v_err > ev);

    theta_mat(i+1, :) = thetalist(:)';  % row for iteration 0

    fprintf('\n================ IKinBodyIterations Report ================\n');
    printIteration(i, thetalist, Tsb, Vb, omg_err, v_err);

    % --- Newton-Raphson iterations ---
    while err && i < maxiterations
        thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
        i = i + 1;

        Tsb = FKinBody(M, Blist, thetalist);
        Vb  = se3ToVec(MatrixLog6(TransInv(Tsb) * T));
        omg_err = norm(Vb(1:3));
        v_err   = norm(Vb(4:6));
        err = (omg_err > eomg) || (v_err > ev);

        theta_mat(i+1, :) = thetalist(:)';  % save this iteration as a row

        printIteration(i, thetalist, Tsb, Vb, omg_err, v_err);
    end

    success = ~err;

    % Trim to the number of iterations actually used (0..i)
    theta_mat = theta_mat(1:i+1, :);

    % Save CSV (each row = joint values for that iteration)
    writematrix(theta_mat, csv_filename);

    fprintf('-----------------------------------------------------------\n');
    fprintf('Finished: success = %d, iterations = %d\n', success, i);
    fprintf('Saved joint iteration history to: %s\n\n', csv_filename);
end

% ---------- helper printer ----------
function printIteration(i, thetalist, Tsb, Vb, omg_err, v_err)
    fprintf('\nIteration %d\n', i);
    fprintf('theta^%d (joint vector):\n', i);
    disp(thetalist);

    fprintf('T_sb(theta^%d) (current end-effector config):\n', i);
    disp(Tsb);

    fprintf('V_b (error twist):\n');
    disp(Vb);

    fprintf('||omega_b|| = %.6f, ||v_b|| = %.6f\n', omg_err, v_err);
end
