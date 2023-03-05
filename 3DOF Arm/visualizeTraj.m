% Visualize  
    subplot(2,2,1)
    hold on
    grid on
    plot([0;0],q_i,'bo');
    plot([tf;tf],q_f,'ro');
    plot(T(1:10:end),q_r0(1:10:end,:),'ko');
    title("Joint Position")
    legend('InitPos','FinalPos','ComputedPos')

    subplot(2,2,2)
    plot(T,dq_r0);
    grid on
    title("Joint Velocity")
    legend('q1','q2')

    subplot(2,2,3)
    plot(T,ddq_r0);
    grid on
    title("Joint Acceleration")
    legend('q1','q2')
   