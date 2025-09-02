% ====== 完整多设备 GA 串口采集 + 注入对比 + 绘图 脚本（COM6 & COM7） ======

% -------- 配置区 --------
ports = ["COM5", "COM7", "COM8", "COM10"];           % 串口列表"COM5", "COM7", "COM8", "COM10"
baudRate = 115200;
numRuns = 30;
maxGen = 5000;
captureInterval = 30;
perRunTimeoutSec = 360;             % 备用（未用于强制整轮超时，仅做 debug 参考）
stallTimeoutSec = 60;               % 若该轮超过这个时间无新行则认为“卡住”超时

global_opt_threshold = 0.00001;        % 阈值：Best-so-far <= 该值认为达成

% 视觉参数
lw_pre  = 1.0;
lw_post = 0.8;
lw_rx   = 1.0;
lw_inj  = 1.0;
markerSize = 4;
reachMarkerSize = 8;

output_folder_base = "GA30_multi_COM";
ts_global = datestr(now, 'yyyymmdd_HHMMSS');

% 正则表达式（匹配 Arduino 输出格式）
seed_pattern = "Seed,0x([0-9A-Fa-f]+),(\d+)";
gen_pattern  = "Gen,(\d+),Best,([\-+]?[0-9]*\.?[0-9]+(?:[eE][\-+]?[0-9]+)?),GenBest,([\-+]?[0-9]*\.?[0-9]+(?:[eE][\-+]?[0-9]+)?),RXtotal=(\d+)";

% ====== 初始化设备结构 ======
nDevices = numel(ports);
devices = struct();

for d = 1:nDevices
    devices(d).DeviceID = d;
    devices(d).Port = ports(d);
    fprintf("打开串口 %s @ %d 波特...\n", ports(d), baudRate);
    devices(d).Serial = serialport(ports(d), baudRate, Timeout=1);
    configureTerminator(devices(d).Serial, "LF");
    flush(devices(d).Serial);
    pause(0.2);

    % per-run 状态
    devices(d).CurrentSeedHex = "unknown";
    devices(d).CurrentSeedDec = NaN;
    devices(d).LastGen = NaN;
    devices(d).FinishedRuns = false(1, numRuns);
    devices(d).RunStartTime = cell(1, numRuns);       % 存 tic token
    devices(d).LastProgressTime = cell(1, numRuns);  % 存上次有意义输出的 tic token

    % 输出目录
    folder_name = sprintf("%s_%s_%s", output_folder_base, ports(d), ts_global);
    outdir = fullfile(pwd, folder_name);
    if ~exist(outdir, 'dir')
        mkdir(outdir);
    end
    devices(d).OutDir = outdir;

    % 文件路径
    devices(d).gaLogCSV = fullfile(outdir, sprintf("ga_run_log_%s.csv", ts_global));
    devices(d).injCSV   = fullfile(outdir, sprintf("injection_events_%s.csv", ts_global));
    devices(d).injSummaryCSV = fullfile(outdir, sprintf("injection_summary_%s.csv", ts_global));
    devices(d).reachSummaryCSV = fullfile(outdir, sprintf("reach_summary_%s.csv", ts_global));
    devices(d).fig1name = fullfile(outdir, sprintf("convergence_segmented_%s.fig", ts_global));
    devices(d).fig2name = fullfile(outdir, sprintf("rxtotal_growth_%s.fig", ts_global));
    devices(d).fig3name = fullfile(outdir, sprintf("inj_occurrence_%s.fig", ts_global));

    % 日志容器
    devices(d).ga_entries = struct("GA_run", {}, "SeedHex", {}, "SeedDec", {}, ...
        "Generation", {}, "Best", {}, "RXtotal", {});
    devices(d).inj_entries = struct("GA_run", {}, "SeedHex", {}, "InjectionGen", {});
end

% ====== 并行采集主循环（含进展停滞超时逻辑） ======
for run = 1:numRuns
    fprintf("\n=== 第 %d 轮 GA：向所有设备并行下发 START ===\n", run);
    for d = 1:nDevices
        devices(d).CurrentSeedHex = "unknown";
        devices(d).CurrentSeedDec = NaN;
        devices(d).LastGen = NaN;
        devices(d).FinishedRuns(run) = false;
        devices(d).RunStartTime{run} = tic;           % 启动时间（调试参考）
        devices(d).LastProgressTime{run} = tic;      % 进展时间初始化
        writeline(devices(d).Serial, "START");
    end

    all_done = false;
    while ~all_done
        all_done = true;
        for d = 1:nDevices
            portName = ports(d);
            if devices(d).FinishedRuns(run)
                continue;
            end
            all_done = false;

            % 检查停滞：如果最后一次有意义输出超过 stallTimeoutSec 则超时
            if isempty(devices(d).LastProgressTime{run})
                devices(d).LastProgressTime{run} = tic;
            end
            stalled = toc(devices(d).LastProgressTime{run}) > stallTimeoutSec;
            if stalled
                warning("设备 %s 第 %d 轮因为 %d 秒无进展 被标记超时。", portName, run, stallTimeoutSec);
                devices(d).FinishedRuns(run) = true;
                continue;
            end

            % 可选：整体运行时间输出（用于 debug，看是否超过 perRunTimeoutSec）
            if ~isempty(devices(d).RunStartTime{run})
                total_elapsed = toc(devices(d).RunStartTime{run});
                if total_elapsed > perRunTimeoutSec
                    fprintf("提示：设备 %s 第 %d 轮总运行已超 %d 秒，但仍有进展，不提前中断。\n", ...
                        portName, run, perRunTimeoutSec);
                end
            end

            s = devices(d).Serial;
            while s.NumBytesAvailable > 0
                try
                    line = readline(s);
                catch
                    break;
                end
                line = strtrim(line);
                if isempty(line)
                    continue;
                end
                fprintf("[Port %s Run %d] %s\n", portName, run, line);

                % 有意义输出，刷新进展时间
                devices(d).LastProgressTime{run} = tic;

                % 解析 seed
                tok_seed = regexp(line, seed_pattern, "tokens");
                if ~isempty(tok_seed)
                    t = tok_seed{1};
                    devices(d).CurrentSeedHex = t{1};
                    devices(d).CurrentSeedDec = str2double(t{2});
                    continue;
                end

                % 解析 generation
                tok_gen = regexp(line, gen_pattern, "tokens");
                if ~isempty(tok_gen)
                    t = tok_gen{1};
                    genNum = str2double(t{1});
                    best_val = str2double(t{2});
                    rx_total = str2double(t{4});
                    prevGen = devices(d).LastGen;
                    devices(d).LastGen = genNum;

                    devices(d).ga_entries(end+1) = struct( ...
                        "GA_run", run, ...
                        "SeedHex", string(devices(d).CurrentSeedHex), ...
                        "SeedDec", devices(d).CurrentSeedDec, ...
                        "Generation", genNum, ...
                        "Best", best_val, ...
                        "RXtotal", rx_total ...
                    ); %#ok<SAGROW>

                    if genNum >= maxGen
                        devices(d).FinishedRuns(run) = true;
                        fprintf("设备 %s 第 %d 轮达到 maxGen=%d，完成。\n", portName, run, maxGen);
                    end

                    if ~isnan(prevGen) && genNum < prevGen
                        fprintf("警告：设备 %s Run %d 代数从 %d 下降到 %d（可能重置）。\n", portName, run, prevGen, genNum);
                    end
                    continue;
                end

                % 解析注入事件
                if contains(line, "INJ")
                    if ~isnan(devices(d).LastGen)
                        devices(d).inj_entries(end+1) = struct( ...
                            "GA_run", run, ...
                            "SeedHex", string(devices(d).CurrentSeedHex), ...
                            "InjectionGen", devices(d).LastGen ...
                        ); %#ok<SAGROW>
                        fprintf("设备 %s Run %d 记录 INJ at Gen %d\n", portName, run, devices(d).LastGen);
                    else
                        devices(d).inj_entries(end+1) = struct( ...
                            "GA_run", run, ...
                            "SeedHex", string(devices(d).CurrentSeedHex), ...
                            "InjectionGen", NaN ...
                        ); %#ok<SAGROW>
                        fprintf("设备 %s Run %d 记录 INJ 但 lastGen NaN\n", portName, run);
                    end
                end
            end
        end
        pause(0.01);
    end
end

% 清串口对象
for d = 1:nDevices
    clear devices(d).Serial;
end

% ====== 后处理：每设备写 CSV + 生成注入摘要 + 阈值摘要 + 绘图 ======
combined_summary = table([], [], [], [], [], [], 'VariableNames', ...
    {'Port','ReachedThreshold','FirstReachGen','TotalInjections','ExpectedCaptures','UnexpectedInjections'});

for d = 1:nDevices
    portName = ports(d);
    fprintf("\n=== 处理设备 %s ===\n", portName);

    % ---- GA 日志 ----
    if ~isempty(devices(d).ga_entries)
        T_ga = struct2table(devices(d).ga_entries);
        T_ga = sortrows(T_ga, ["GA_run","Generation"]);
        T_ga.GA_run = double(T_ga.GA_run);
        T_ga.Generation = double(T_ga.Generation);
        T_ga.Best = double(T_ga.Best);
        T_ga.RXtotal = double(T_ga.RXtotal);
        writetable(T_ga, devices(d).gaLogCSV);
        fprintf("写入 GA 日志: %s (%d 行)\n", devices(d).gaLogCSV, height(T_ga));
    else
        warning("设备 %s 没有 GA 日志。", portName);
        T_ga = table([], [], [], [], [], [], 'VariableNames', ...
            {'GA_run','SeedHex','SeedDec','Generation','Best','RXtotal'});
    end

    % ---- 原始注入 ----
    if ~isempty(devices(d).inj_entries)
        T_inj = struct2table(devices(d).inj_entries);
        T_inj.GA_run = double(T_inj.GA_run);
        T_inj.InjectionGen = double(T_inj.InjectionGen);
        writetable(T_inj, devices(d).injCSV);
        fprintf("写入 原始注入日志: %s (%d 行)\n", devices(d).injCSV, height(T_inj));
    else
        warning("设备 %s 没有 注入日志。", portName);
        T_inj = table([], [], [], 'VariableNames', {'GA_run','SeedHex','InjectionGen'});
    end

    % ---- 构造期望注入摘要 ----
    expected_caps = captureInterval:captureInterval:maxGen;
    runs = unique(T_ga.GA_run);
    inj_summary = struct("GA_run", {}, "SeedHex", {}, "ExpectedCaptureGen", {}, ...
        "ActualInjectionGen", {}, "InjectionOccurred", {});
    for i_run = 1:numel(runs)
        runid = runs(i_run);
        sub_ga = T_ga(T_ga.GA_run == runid, :);
        if ismember("SeedHex", sub_ga.Properties.VariableNames)
            seeds = unique(sub_ga.SeedHex);
            seedLabel = "unknown";
            if ~isempty(seeds) && strlength(seeds{1})>0
                seedLabel = seeds(1);
            end
        else
            seedLabel = "unknown";
        end
        for ec = expected_caps
            inj_summary(end+1) = struct( ...
                "GA_run", runid, ...
                "SeedHex", seedLabel, ...
                "ExpectedCaptureGen", ec, ...
                "ActualInjectionGen", NaN, ...
                "InjectionOccurred", 0 ...
            ); %#ok<SAGROW>
        end
    end
    T_summary = struct2table(inj_summary);

    % ---- 填充实际注入 ----
    if ~isempty(devices(d).inj_entries)
        T_real = struct2table(devices(d).inj_entries);
        T_real.GA_run = double(T_real.GA_run);
        if ismember("SeedHex", T_real.Properties.VariableNames)
            T_real.SeedHex = string(T_real.SeedHex);
        else
            T_real.SeedHex = repmat("unknown", height(T_real), 1);
        end
        T_real.InjectionGen = double(T_real.InjectionGen);
        for k = 1:height(T_real)
            row = T_real(k,:);
            runid = row.GA_run;
            injGen = row.InjectionGen;
            seedLabel = row.SeedHex;
            idx = find(T_summary.GA_run==runid & T_summary.ExpectedCaptureGen==injGen);
            if ~isempty(idx) && ~isnan(injGen)
                T_summary.ActualInjectionGen(idx) = injGen;
                T_summary.InjectionOccurred(idx) = 1;
            else
                extra = table(runid, seedLabel, NaN, injGen, 1, ...
                    'VariableNames', {'GA_run','SeedHex','ExpectedCaptureGen','ActualInjectionGen','InjectionOccurred'});
                T_summary = [T_summary; extra]; %#ok<AGROW>
            end
        end
    end
    T_inj_final = sortrows(T_summary, ["GA_run","ExpectedCaptureGen"], 'MissingPlacement','last');
    writetable(T_inj_final, devices(d).injSummaryCSV);
    fprintf("写入 注入摘要日志: %s (%d 行)\n", devices(d).injSummaryCSV, height(T_inj_final));

    % ---- 计算阈值抵达 ----
    reach_info = table('Size',[0 4], ...
        'VariableTypes',["double","string","double","logical"], ...
        'VariableNames',["GA_run","SeedHex","ReachGen","Reached"]);
    for i_run = 1:numel(runs)
        runid = runs(i_run);
        sub = T_ga(T_ga.GA_run==runid,:);
        if isempty(sub)
            continue;
        end
        [~, ord] = sort(sub.Generation);
        best_vals = sub.Best(ord);
        best_so_far = cummin(best_vals);
        idx = find(best_so_far <= global_opt_threshold, 1, 'first');
        if ~isempty(idx)
            reachGen = sub.Generation(ord(idx));
            reached = true;
        else
            reachGen = NaN;
            reached = false;
        end
        if ismember("SeedHex", sub.Properties.VariableNames)
            seeds = unique(sub.SeedHex);
            seedLabel = "unknown";
            if ~isempty(seeds) && strlength(seeds{1})>0
                seedLabel = seeds(1);
            end
        else
            seedLabel = "unknown";
        end
        reach_info = [reach_info; {runid, seedLabel, reachGen, reached}]; %#ok<AGROW>
    end
    writetable(reach_info, devices(d).reachSummaryCSV);
    fprintf("写入 阈值抵达摘要: %s (%d 行)\n", devices(d).reachSummaryCSV, height(reach_info));
    disp(reach_info);

    % ---- 绘图：收敛曲线 ----
    fig1 = figure('Name',sprintf('Convergence_%s', portName),'NumberTitle','off');
    hold on;
    legendEntries = strings(numel(runs),1);
    colors = lines(numel(runs));
    for i = 1:numel(runs)
        runid = runs(i);
        sub = T_ga(T_ga.GA_run==runid,:);
        if isempty(sub), continue; end
        [~, ord] = sort(sub.Generation);
        gens = sub.Generation(ord);
        best_vals = sub.Best(ord);
        best_so_far = cummin(best_vals);
        idx = find(best_so_far <= global_opt_threshold, 1, 'first');
        baseColor = colors(i,:);
        fadedColor = baseColor + (1 - baseColor) * 0.6;
        fadedColor(fadedColor>1) = 1;

        if ~isempty(idx)
            plot(gens(1:idx), best_vals(1:idx), '-', 'LineWidth', lw_pre, 'Color', baseColor);
            if idx < numel(gens)
                plot(gens(idx+1:end), best_vals(idx+1:end), '--', 'LineWidth', lw_post, 'Color', fadedColor);
            end
            plot(gens(idx), best_vals(idx), 'p', 'MarkerSize', reachMarkerSize, ...
                'MarkerFaceColor', baseColor, 'MarkerEdgeColor','k');
        else
            plot(gens, best_vals, '-', 'LineWidth', lw_pre, 'Color', baseColor);
        end
        legendEntries(i) = sprintf("Run %d", runid);
    end
    yline(global_opt_threshold, '--k', sprintf("Threshold=%.3g", global_opt_threshold), ...
        'LabelHorizontalAlignment','left');
    xlabel('Generation'); ylabel('Best Objective');
    title(sprintf('收敛曲线（%s）', portName));
    legend(legendEntries, 'Location','eastoutside','Interpreter','none');
    grid on; hold off;
    set(fig1,'Position',[100 100 1100 550]);
    savefig(fig1, devices(d).fig1name);
    close(fig1);

    % ---- 绘图：RXtotal 增长 ----
    if ismember("RXtotal", T_ga.Properties.VariableNames)
        fig2 = figure('Name',sprintf('RXtotal_%s', portName),'NumberTitle','off');
        hold on;
        for i = 1:numel(runs)
            runid = runs(i);
            sub = T_ga(T_ga.GA_run==runid,:);
            if isempty(sub), continue; end
            [~, ord] = sort(sub.Generation);
            plot(sub.Generation(ord), sub.RXtotal(ord), '-', 'LineWidth', lw_rx, 'DisplayName', sprintf('Run %d', runid));
        end
        xlabel('Generation'); ylabel('RXtotal');
        title(sprintf('RXtotal 增长（%s）', portName));
        legend('Location','bestoutside');
        grid on; hold off;
        set(fig2,'Position',[100 100 1000 500]);
        savefig(fig2, devices(d).fig2name);
        close(fig2);
    end

    % ---- 绘图：注入事件（期望 vs 预期外） ----
    fig3 = figure('Name',sprintf('INJ_%s', portName),'NumberTitle','off');
    hold on;
    for i = 1:numel(runs)
        runid = runs(i);
        sub_exp = T_inj_final(T_inj_final.GA_run==runid & ~isnan(T_inj_final.ExpectedCaptureGen), :);
        if ~isempty(sub_exp)
            x = sub_exp.ExpectedCaptureGen;
            y = sub_exp.InjectionOccurred;
            plot(x, y, '-o', 'LineWidth', lw_inj, 'MarkerSize', markerSize, 'DisplayName', sprintf('Run %d (期望)', runid));
        end
        sub_extra = T_inj_final(T_inj_final.GA_run==runid & isnan(T_inj_final.ExpectedCaptureGen) & T_inj_final.InjectionOccurred==1, :);
        if ~isempty(sub_extra)
            x_extra = sub_extra.ActualInjectionGen;
            x_extra = x_extra(~isnan(x_extra));
            scatter(x_extra, ones(size(x_extra)), 50, 'x', 'LineWidth',1.5, 'DisplayName', sprintf('Run %d 预期外', runid));
        end
    end
    xlabel('Generation');
    ylabel('Injection Occurred (0/1)');
    title(sprintf('注入事件（每 %d 代）（%s）', captureInterval, portName));
    ylim([-0.1 1.2]);
    yticks([0 1]);
    grid on; legend('Location','bestoutside');
    hold off;
    set(fig3,'Position',[100 100 1000 500]);
    savefig(fig3, devices(d).fig3name);
    close(fig3);

    % ---- 汇总统计（combined summary） ----
    reached_any = any(reach_info.Reached);
    first_reach = NaN;
    if reached_any
        achieved = reach_info(reach_info.Reached==1,:);
        valid_gens = achieved.ReachGen(~isnan(achieved.ReachGen));
        if ~isempty(valid_gens)
            first_reach = min(valid_gens);
        end
    end
    total_inj = height(T_inj_final(T_inj_final.InjectionOccurred==1,:));
    expected_caps_count = numel(expected_caps) * numel(runs);
    matched = T_inj_final(T_inj_final.InjectionOccurred==1 & ~isnan(T_inj_final.ExpectedCaptureGen), :);
    unexpected_inj = total_inj - height(matched);

    combined_summary = [combined_summary; {
        portName, ...
        reached_any, ...
        first_reach, ...
        total_inj, ...
        expected_caps_count, ...
        unexpected_inj}];
end

% ====== 写整体 summary ======
combined_summary.Properties.VariableNames = {'Port','ReachedThreshold','FirstReachGen','TotalInjections','ExpectedCaptures','UnexpectedInjections'};
combined_summary_csv = fullfile(pwd, sprintf("combined_summary_%s.csv", ts_global));
writetable(combined_summary, combined_summary_csv);
fprintf("\n写入整体 summary: %s\n", combined_summary_csv);
disp(combined_summary);

fprintf("\n=== 全部完成 ===\n");
