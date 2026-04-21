clc; clear; close all;

% ===== 参数 =====
alpha_p = 8.5; % Peak alpha (deg)
h = linspace(0, 60, 1000); % 高度 km

% ===== smoothstep函数 =====
smoothstep = @(x) max(0,min(1, x.^2 .* (3 - 2*x)));

% ===== 1. Baseline constant =====
alpha1 = zeros(size(h));
alpha1(h < 30) = alpha_p;

% ===== 2. Smooth glide =====
alpha2 = zeros(size(h));
alpha_low = 0.65 * alpha_p;

for i = 1:length(h)
    if h(i) >= 30
        alpha2(i) = 0;
    elseif h(i) >= 20
        t = (30 - h(i)) / 10;
        alpha2(i) = alpha_p * smoothstep(t);
    elseif h(i) >= 10
        alpha2(i) = alpha_p;
    else
        t = (10 - h(i)) / 10;
        alpha2(i) = alpha_p - (alpha_p - alpha_low)*smoothstep(t);
    end
end

% ===== 3. Qian-style =====
alpha3 = zeros(size(h));
for i = 1:length(h)
    if h(i) >= 45
        alpha3(i) = 0;
    elseif h(i) >= 30
        t = (45 - h(i)) / 15;
        alpha3(i) = 0.45 * alpha_p * smoothstep(t);
    elseif h(i) >= 18
        t = (30 - h(i)) / 12;
        alpha3(i) = 0.45*alpha_p + (1.35-0.45)*alpha_p*smoothstep(t);
    elseif h(i) >= 12
        alpha3(i) = 1.35 * alpha_p;
    elseif h(i) >= 4
        t = (12 - h(i)) / 8;
        alpha3(i) = 1.35*alpha_p - (1.35-0.75)*alpha_p*smoothstep(t);
    else
        alpha3(i) = 0.75 * alpha_p;
    end
end

% ===== 4. Aggressive skip =====
alpha4 = zeros(size(h));
for i = 1:length(h)
    if h(i) >= 55
        alpha4(i) = 0;
    elseif h(i) >= 38
        t = (55 - h(i)) / 17;
        alpha4(i) = 0.35 * alpha_p * smoothstep(t);
    elseif h(i) >= 24
        t = (38 - h(i)) / 14;
        alpha4(i) = 0.35*alpha_p + (1.60-0.35)*alpha_p*smoothstep(t);
    elseif h(i) >= 16
        alpha4(i) = 1.60 * alpha_p;
    elseif h(i) >= 8
        t = (16 - h(i)) / 8;
        alpha4(i) = 1.60*alpha_p - (1.60-0.95)*alpha_p*smoothstep(t);
    else
        alpha4(i) = 0.95 * alpha_p;
    end
end

%% ===== 单独四张图 =====
figure; plot(alpha1, h); grid on;
xlabel('\alpha (deg)'); ylabel('Height (km)');
title('Baseline Constant');

figure; plot(alpha2, h); grid on;
xlabel('\alpha (deg)'); ylabel('Height (km)');
title('Smooth Glide');

figure; plot(alpha3, h); grid on;
xlabel('\alpha (deg)'); ylabel('Height (km)');
title('Aggressive Skip');

%% ===== 综合对比图 =====
figure;
plot(alpha1, h); hold on;
plot(alpha2, h);
plot(alpha3, h);
grid on;

xlabel('\alpha (deg)');
ylabel('Height (km)');
title('Alpha Profiles Comparison');

legend('Baseline','Smooth Glide','Aggressive Skip');