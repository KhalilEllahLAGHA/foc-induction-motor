%FOC_INIT  Initialize the workspace for the FOC Simulink models.
%
%   Defines the parameter struct P (see motor_params.m) that every block in
%   foc_im_blocks.slx / foc_im_library.slx references, and prints a short
%   summary of the derived controller design.
%
%   Run this (or one of the run_foc_* scripts, which call it) before
%   simulating either model.

P = motor_params();

fprintf('FOC induction motor - parameters loaded into struct P\n');
fprintf('  machine        : %.0f kW, %.0f V, %.0f Hz, %d pole pairs\n', ...
        P.Pn/1e3, P.Vn, P.fn, P.p);
fprintf('  sigma          : %.4f    tau_r: %.3f s\n', P.sigma, P.tau_r);
fprintf('  current PI     : Kp = %.2f V/A,  Ki = %.0f V/(A s)  (BW %.0f Hz)\n', ...
        P.Kp_i, P.Ki_i, P.bw_i/(2*pi));
fprintf('  speed PI       : Kp = %.3f,  Ki = %.2f  (BW %.0f Hz)\n', ...
        P.Kp_w, P.Ki_w, P.bw_w/(2*pi));
fprintf('  references     : psi* = %.2f Wb (id* = %.2f A)\n', ...
        P.psi_ref, P.id_ref);
fprintf('  scenario       : speed steps to [%s] rpm\n', ...
        strtrim(sprintf('%d ', P.step_rpm)));
fprintf('                   %.0f N*m load at %.2f s, T_end = %.2f s\n', ...
        P.TL, P.t_load, P.T_end);
