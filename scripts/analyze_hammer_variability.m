function outDir = analyze_hammer_variability(dataRoot, outDir)
% ANALYZE_HAMMER_VARIABILITY Offline, base-MATLAB analysis of recorded strikes.
% From the repository root:
%   addpath('scripts'); analyze_hammer_variability;
% Optional arguments: directory containing all_trials.csv/groups.json, output.
% Reads original CSV/JSON only. Does not connect to hardware or change inputs.
repo = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1 || isempty(dataRoot)
    dataRoot = fullfile(repo,'outputs','hammer_experiment_20261020');
end
if nargin < 2 || isempty(outDir)
    outDir = fullfile(repo,'outputs', ...
        ['matlab_variability_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))]);
end
if ~isfolder(outDir), mkdir(outDir); end
rng(20261020,'twister');
bootstrapN = 2000;
source = readtable(fullfile(dataRoot,'all_trials.csv'),'TextType','string');
groups = jsondecode(fileread(fullfile(dataRoot,'groups.json')));
if iscell(groups)
    groupDirs = string(cellfun(@(g) g.directory,groups,'UniformOutput',false));
else
    groupDirs = string({groups.directory});
end
for i = 1:numel(groupDirs)
    groupDirs(i) = resolveArchivePath(dataRoot,groupDirs(i));
end
assert(numel(unique(source.result_file)) == height(source),'Duplicate result files.');
records = cell(height(source),1); waves = cell(height(source),1);
for i = 1:height(source)
    path = resolveArchivePath(dataRoot,source.result_file(i));
    d = jsondecode(fileread(path)); r = d.row;
    folder = fileparts(path);
    assert(any(groupDirs == fileparts(fileparts(folder))), ...
        'Result file is outside selected groups.json.');
    a = readtable(fullfile(folder,'accel.csv'));
    [~,ia] = unique(a.seq_unwrapped,'sorted'); a = a(ia,:);
    action = a.seq_unwrapped > d.accel_fire.sample_seq;
    assert(any(action),'Missing action samples.');
    rawAxes = a{:,{'x','y','z'}};
    dynamic = sqrt(sum(rawAxes.^2,2));
    % ADXL375 right-justified code rails, with D0=0 at 1600 Hz.
    % Flag original axes, not the vector magnitude.
    railSamples = any(rawAxes>=4094 | rawAxes<=-4096,2) & action;
    nearRail = any(abs(rawAxes)>=4000,2) & action;
    actionIdx = find(action);
    [peak,j] = max(dynamic(action)); peakIdx = actionIdx(j);
    assert(abs(peak-r.accel_raw_peak_counts) < 1e-6,'Raw acceleration peak mismatch.');
    tr = readtable(fullfile(folder,'trace.csv'));
    speed = NaN; cutoff = NaN; steadyMean = NaN;
    if isfield(d,'encoder_metrics') && isfield(d.encoder_metrics,'cutoff_us')
        cutoff = d.encoder_metrics.cutoff_us;
        k = find(tr.time_us < cutoff & isfinite(tr.speed),1,'last');
        assert(~isempty(k),'No pre-drop speed sample.');
        speed = tr.speed(k);
        assert(abs(speed-r.pre_drop_speed_dps) < 1e-6,'Raw speed mismatch.');
        steadyMean = numericField(d.encoder_metrics,'steady_mean');
    end
    records{i} = struct('target_dps',r.speed_dps,'grade_trial',source.grade_trial(i), ...
        'shot',r.shot,'speed_dps',speed,'steady_mean_dps',steadyMean, ...
        'accel_peak_counts',peak,'speed_error_pct',100*(speed/r.speed_dps-1), ...
        'completed',strcmp(r.fire_state,'completed'), ...
        'window_complete',logicalField(r,'accel_window_complete'), ...
        'qualified',logicalField(r,'quality_qualified'), ...
        'speed_qualified',logicalField(r,'speed_qualified'), ...
        'multi_event',logicalField(r,'accel_multi_event'), ...
        'link_fault',logicalField(r,'accel_link_fault'), ...
        'early_drop',logicalField(r,'early_drop_suspected'), ...
        'gap_count',numericField(r,'accel_gap_count'), ...
        'rail_hit',any(railSamples),'rail_sample_count',sum(railSamples), ...
        'near_rail',any(nearRail),'peak_at_rail',railSamples(peakIdx), ...
        'reason',string(r.reason),'session',source.session(i),'result_file',path);
    waves{i} = struct('speed_time_ms',tr.time_us/1000,'speed',tr.speed, ...
        'cutoff_ms',cutoff/1000,'accel_time_ms', ...
        (a.time_unwrapped_us-a.time_unwrapped_us(peakIdx))/1000, ...
        'accel',dynamic,'raw_axes',rawAxes, ...
        'accel_peak_seq',a.seq_unwrapped(peakIdx));
end
T = struct2table(vertcat(records{:}));
[T,order] = sortrows(T,{'target_dps','grade_trial'}); waves = waves(order);
assert(height(unique(T(:,{'session','shot'}))) == height(T),'Duplicate FIRE IDs.');
speeds = unique(T.target_dps)';
subsetNames = ["all_recorded","normal_complete","quality_qualified","complete_no_rail"];
statRows = cell(numel(speeds)*numel(subsetNames),1);
assocRows = statRows; rowNumber = 0;
for target = speeds
    g = T.target_dps == target;
    for subset = subsetNames
        mask = g & subsetMask(T,subset);
        v = T.speed_dps(mask); a = T.accel_peak_counts(mask);
        vs = describe(v,bootstrapN); as = describe(a,bootstrapN);
        rowNumber = rowNumber+1;
        statRows{rowNumber} = struct('target_dps',target,'subset',subset, ...
            'n_records',sum(mask),'n_speed',vs.n,'speed_mean_dps',vs.mean, ...
            'speed_sd_dps',vs.sd,'speed_cv_pct',vs.cv, ...
            'speed_cv_ci_low',vs.lo,'speed_cv_ci_high',vs.hi, ...
            'speed_bias_pct',100*(vs.mean/target-1), ...
            'speed_rmse_dps',sqrt(mean((v(isfinite(v))-target).^2)), ...
            'n_accel',as.n,'accel_mean_counts',as.mean,'accel_sd_counts',as.sd, ...
            'accel_cv_pct',as.cv,'accel_cv_ci_low',as.lo,'accel_cv_ci_high',as.hi, ...
            'n_multi_event',sum(T.multi_event(mask)), ...
            'n_incomplete',sum(~T.window_complete(mask)), ...
            'n_aborted',sum(~T.completed(mask)));
        statRows{rowNumber}.n_rail_hit = sum(T.rail_hit(mask));
        statRows{rowNumber}.n_near_rail = sum(T.near_rail(mask));
        valid = mask & isfinite(T.speed_dps) & isfinite(T.accel_peak_counts);
        assocRows{rowNumber} = struct('target_dps',target,'subset',subset, ...
            'n_pairs',sum(valid),'pearson_r',pearson(T.speed_dps(valid),T.accel_peak_counts(valid)));
    end
end
S = struct2table(vertcat(statRows{:})); R = struct2table(vertcat(assocRows{:}));
writetable(T,fullfile(outDir,'trial_metrics.csv'));
writetable(S,fullfile(outDir,'grade_statistics.csv'));
writetable(R,fullfile(outDir,'within_grade_correlations.csv'));
copyfile(fullfile(dataRoot,'groups.json'),fullfile(outDir,'source_groups.json'));
accelDefinition='raw_vector_norm';
save(fullfile(outDir,'analysis.mat'),'T','S','R','waves','dataRoot','bootstrapN','accelDefinition');

% Figures use only base MATLAB; no Statistics Toolbox is required.
oldVisible = get(groot,'defaultFigureVisible');
cleanup = onCleanup(@() set(groot,'defaultFigureVisible',oldVisible));
set(groot,'defaultFigureVisible','off');
colors = lines(numel(speeds));
A = S(S.subset=="all_recorded",:);
B = S(S.subset=="normal_complete",:);
Q = S(S.subset=="quality_qualified",:);
U = S(S.subset=="complete_no_rail",:);
f = figure('Color','w','Position',[80 80 1420 850]);
tl = tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
ax = nexttile(tl); hold(ax,'on');
plot(ax,[min(speeds)-40 max(speeds)+40],[min(speeds)-40 max(speeds)+40], ...
    '--','Color',[.6 .6 .6],'DisplayName','Target');
scatterByGrade(ax,T,T.speed_dps,colors);
errorbar(ax,A.target_dps,A.speed_mean_dps,A.speed_sd_dps,'ko-', ...
    'LineWidth',1.5,'MarkerFaceColor','w','DisplayName','Mean +/- sample SD');
xlabel(ax,'Target angular speed (deg/s)'); ylabel(ax,'Pre-drop angular speed (deg/s)');
title(ax,'Angular-speed repeatability'); legend(ax,'Location','northwest');
ax = nexttile(tl); hold(ax,'on'); scatterByGrade(ax,T,T.accel_peak_counts,colors);
errorbar(ax,A.target_dps,A.accel_mean_counts,A.accel_sd_counts,'ko-', ...
    'LineWidth',1.5,'MarkerFaceColor','w','DisplayName','Mean +/- sample SD');
xlabel(ax,'Target angular speed (deg/s)'); ylabel(ax,'Acceleration peak (counts)');
title(ax,'Raw three-axis acceleration magnitude'); legend(ax,'Location','northwest');
ax = nexttile(tl); hold(ax,'on');
plot(ax,A.target_dps,A.speed_cv_pct,'o-','LineWidth',1.6,'DisplayName','Angular speed');
plot(ax,A.target_dps,A.accel_cv_pct,'s-','LineWidth',1.6,'DisplayName','Acceleration peak');
for k=1:height(A)
    text(ax,A.target_dps(k),A.accel_cv_pct(k)+.7,sprintf('%.1f%%',A.accel_cv_pct(k)), ...
        'HorizontalAlignment','center','FontSize',10);
end
xlabel(ax,'Target angular speed (deg/s)'); ylabel(ax,'CV (%)');
title(ax,'Between-strike variation: sample SD / mean'); legend(ax,'Location','northwest');
ax = nexttile(tl); hold(ax,'on'); yline(ax,0,'k-','HandleVisibility','off');
yline(ax,10,'--','Color',[.7 .7 .7],'HandleVisibility','off');
yline(ax,-10,'--','Color',[.7 .7 .7],'HandleVisibility','off');
scatterByGrade(ax,T,T.speed_error_pct,colors);
errorbar(ax,A.target_dps,A.speed_bias_pct,100*A.speed_sd_dps./A.target_dps, ...
    'ko-','LineWidth',1.5,'MarkerFaceColor','w','DisplayName','Mean +/- sample SD');
xlabel(ax,'Target angular speed (deg/s)'); ylabel(ax,'Angular-speed error (%)');
title(ax,'Target accuracy is separate from repeatability');
title(tl,{sprintf('Recorded strikes | n=%d | circles: qualified; x: flagged',height(T)), ...
    'Hollow diamonds: ADC rail hit; high-grade acceleration CV is affected by clipping'}, ...
    'FontSize',16,'FontWeight','bold');
formatAxes(f,speeds); exportFigure(f,outDir,'01_variability_overview');

f = figure('Color','w','Position',[80 80 1420 640]);
tl = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
for metric = 1:2
    ax = nexttile(tl); hold(ax,'on');
    if metric==1, key='speed_cv_pct'; low='speed_cv_ci_low'; high='speed_cv_ci_high'; label='Angular speed';
    else, key='accel_cv_pct'; low='accel_cv_ci_low'; high='accel_cv_ci_high'; label='Acceleration peak'; end
    errorbar(ax,A.target_dps-12,A.(key),A.(key)-A.(low),A.(high)-A.(key), ...
        'o-','LineWidth',1.3,'DisplayName','All recorded: 95% bootstrap interval');
    plot(ax,B.target_dps,B.(key),'s--','LineWidth',1.4,'DisplayName','Normal + complete window');
    plot(ax,Q.target_dps+12,Q.(key),'d:','LineWidth',1.5,'DisplayName','Composite quality subset');
    plot(ax,U.target_dps+22,U.(key),'^:','LineWidth',1.3,'DisplayName','Complete without rail hit (selected subset)');
    xlabel(ax,'Target angular speed (deg/s)'); ylabel(ax,'CV (%)'); title(ax,label);
    legend(ax,'Location','northwest');
end
title(tl,{'Subset sensitivity and CV uncertainty', ...
    '300 deg/s: zero composite-qualified trials; rail-free subset does not recover clipped impacts'},'FontSize',15);
formatAxes(f,speeds); exportFigure(f,outDir,'02_cv_sensitivity');

f = figure('Color','w','Position',[80 80 1500 760]);
tl = tiledlayout(f,2,numel(speeds),'TileSpacing','compact','Padding','compact');
for row=1:2
    for k=1:numel(speeds)
        ax = nexttile(tl); hold(ax,'on'); idx = T.target_dps==speeds(k);
        if row==1, values=T.speed_dps(idx); center=A.speed_mean_dps(k);
        else, values=T.accel_peak_counts(idx); center=A.accel_mean_counts(k); end
        x=T.grade_trial(idx); qualified=T.qualified(idx);
        plot(ax,x,100*(values/center-1),'-','Color',[.75 .75 .75],'HandleVisibility','off');
        scatter(ax,x(qualified),100*(values(qualified)/center-1),22,colors(k,:),'filled');
        scatter(ax,x(~qualified),100*(values(~qualified)/center-1),35,colors(k,:),'x');
        yline(ax,0,'k:'); xlim(ax,[0 21]); xticks(ax,[1 10 20]);
        if row==1, ylim(ax,[-15 15]); title(ax,sprintf('%d deg/s',speeds(k)));
        else, ylim(ax,[-55 55]); xlabel(ax,'Strike #'); end
        if k==1
            if row==1, ylabel(ax,{'Angular speed','Deviation from grade mean (%)'});
            else, ylabel(ax,{'Acceleration peak','Deviation from grade mean (%)'}); end
        end
        id=find(idx); sessions=T.session(idx); switches=find(sessions(2:end)~=sessions(1:end-1))+1;
        for s=switches', xline(ax,T.grade_trial(id(s))-.5,'--','Color',[.4 .4 .4]); end
    end
end
title(tl,{'Trial sequence: normalized within each grade', ...
    'Vertical dashed lines: session changes; 500 deg/s strike 3 has no valid speed estimate'},'FontSize',15);
formatAxes(f,[]); exportFigure(f,outDir,'03_trial_sequence');

f = figure('Color','w','Position',[80 80 1420 650]);
tl = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
ax1 = nexttile(tl); hold(ax1,'on'); ax2 = nexttile(tl); hold(ax2,'on');
resSpeed=NaN(height(T),1); resAccel=resSpeed;
for k=1:numel(speeds)
    idx=T.target_dps==speeds(k) & isfinite(T.speed_dps) & isfinite(T.accel_peak_counts);
    x=T.speed_dps(idx); y=T.accel_peak_counts(idx); good=T.qualified(idx);
    rx=100*(x/mean(x)-1); ry=100*(y/mean(y)-1);
    resSpeed(idx)=rx; resAccel(idx)=ry;
    label=sprintf('%d deg/s (n=%d, r=%.2f)',speeds(k),sum(idx),pearson(x,y));
    scatter(ax1,x(good),y(good),30,colors(k,:),'filled','DisplayName',label);
    scatter(ax1,x(~good),y(~good),40,colors(k,:),'x','HandleVisibility','off');
    scatter(ax2,rx(good),ry(good),30,colors(k,:),'filled','DisplayName',label);
    scatter(ax2,rx(~good),ry(~good),40,colors(k,:),'x','HandleVisibility','off');
end
xlabel(ax1,'Pre-drop angular speed (deg/s)'); ylabel(ax1,'Acceleration peak (counts)');
title(ax1,'Paired observations: color indicates target grade'); legend(ax1,'Location','northwest');
xlabel(ax2,'Angular-speed deviation from grade mean (%)');
ylabel(ax2,'Acceleration-peak deviation from grade mean (%)');
withinR=pearson(resSpeed,resAccel);
title(ax2,sprintf('Grade-centered association: pooled r=%.3f',withinR));
xline(ax2,0,'k:'); yline(ax2,0,'k:');
title(tl,{'Speed and acceleration association', ...
    'Within-grade associations are descriptive; ordering and session effects are not removed'},'FontSize',15);
formatAxes(f,[]); exportFigure(f,outDir,'04_speed_acceleration');

for kind = ["speed","accel"]
    f = figure('Color','w','Position',[80 80 1500 820]);
    tl = tiledlayout(f,2,4,'TileSpacing','compact','Padding','compact');
    for k=1:numel(speeds)
        ax=nexttile(tl); hold(ax,'on'); ids=find(T.target_dps==speeds(k));
        for i=ids'
            c=.70*colors(k,:)+.30;
            if ~T.qualified(i), c=[.85 .48 .18]; end
            w=waves{i};
            if kind=="speed", x=w.speed_time_ms; y=w.speed;
            else, x=w.accel_time_ms; y=w.accel; end
            plot(ax,x,y,'Color',c,'LineWidth',.7);
        end
        title(ax,sprintf('%d deg/s | n=%d | rail hits=%d',speeds(k),numel(ids),sum(T.rail_hit(ids))));
        if kind=="speed"
            yline(ax,speeds(k),'k--'); xlim(ax,[0 260]); ylim(ax,[0 1000]);
            xlabel(ax,'Time from motor action (ms)'); ylabel(ax,'Angular speed (deg/s)');
        else
            xlim(ax,[-150 100]); ylim(ax,[0 5000]);
            xlabel(ax,'Time from acceleration peak (ms)'); ylabel(ax,'Raw magnitude (counts)');
        end
    end
    ax=nexttile(tl); axis(ax,'off');
    text(ax,.02,.85,{'Orange: composite quality flag', ...
        'No interpolation or smoothing', ...
        'Speed: motor time, 1 ms trace', ...
        'Acceleration: aligned at own peak', ...
        'The clocks are not shared', ...
        'Do not infer precise impact latency'},'Units','normalized','FontSize',12,'VerticalAlignment','top');
    title(tl,sprintf('%s waveform overlays | all selected records',kind),'FontSize',16);
    formatAxes(f,[]);
    if kind=="speed", name='05_speed_waveforms'; else, name='06_acceleration_waveforms'; end
    exportFigure(f,outDir,name);
end
f=figure('Color','w','Position',[80 80 1000 520]);
ax=axes(f); hold(ax,'on');
bar(ax,A.target_dps,100*A.n_rail_hit./A.n_records,.65,'FaceColor',[.8 .3 .15]);
for k=1:height(A)
    text(ax,A.target_dps(k),100*A.n_rail_hit(k)/A.n_records(k)+3, ...
        sprintf('%d/%d',A.n_rail_hit(k),A.n_records(k)),'HorizontalAlignment','center');
end
xlabel(ax,'Target angular speed (deg/s)'); ylabel(ax,'Strikes with an axis at ADC rail (%)');
ylim(ax,[0 100]); xticks(ax,speeds); title(ax,'Raw-axis rail hits after FIRE');
title(ax,'Strikes with raw-axis ADC rail hits');
formatAxes(f,[]); exportFigure(f,outDir,'07_acceleration_clipping');
writeReport(outDir,dataRoot,T,S,R,withinR,bootstrapN);
fprintf('MATLAB %s\nSaved analysis to: %s\n',version,outDir);
disp(A(:,{'target_dps','n_speed','speed_mean_dps','speed_sd_dps','speed_cv_pct', ...
    'n_accel','accel_mean_counts','accel_sd_counts','accel_cv_pct'}));
end

function value=numericField(s,key)
value=NaN;
if isfield(s,key) && isnumeric(s.(key)) && isscalar(s.(key)), value=double(s.(key)); end
end
function value=logicalField(s,key)
value=isfield(s,key) && ~isempty(s.(key)) && logical(s.(key));
end
function mask=subsetMask(T,name)
switch name
    case "all_recorded", mask=true(height(T),1);
    case "normal_complete", mask=T.completed & T.window_complete;
    case "quality_qualified", mask=T.qualified;
    case "complete_no_rail", mask=T.completed & T.window_complete & ~T.rail_hit;
end
end
function s=describe(values,B)
v=values(isfinite(values)); n=numel(v);
s=struct('n',n,'mean',NaN,'sd',NaN,'cv',NaN,'lo',NaN,'hi',NaN);
if n==0, return; end
s.mean=mean(v);
if n<2 || abs(s.mean)<eps, return; end
s.sd=std(v,0); s.cv=100*s.sd/abs(s.mean);
samples=v(randi(n,n,B)); cv=100*std(samples,0,1)./abs(mean(samples,1));
cv=sort(cv(isfinite(cv)));
s.lo=interp1(1:numel(cv),cv,1+(numel(cv)-1)*.025);
s.hi=interp1(1:numel(cv),cv,1+(numel(cv)-1)*.975);
end
function r=pearson(x,y)
valid=isfinite(x)&isfinite(y); x=x(valid); y=y(valid); r=NaN;
if numel(x)<3 || std(x)==0 || std(y)==0, return; end
c=corrcoef(x,y); r=c(1,2);
end
function scatterByGrade(ax,T,y,colors)
speeds=unique(T.target_dps)';
for k=1:numel(speeds)
    idx=T.target_dps==speeds(k); x=T.target_dps(idx)+linspace(-17,17,sum(idx))';
    v=y(idx); q=T.qualified(idx);
    scatter(ax,x(q),v(q),23,colors(k,:),'filled','HandleVisibility','off');
    scatter(ax,x(~q),v(~q),32,colors(k,:),'x','HandleVisibility','off');
    rail=T.rail_hit(idx);
    scatter(ax,x(rail),v(rail),65,'k','d','HandleVisibility','off');
end
end
function formatAxes(f,speeds)
axs=findall(f,'Type','axes');
for ax=axs'
    ax.FontName='Helvetica'; ax.FontSize=11; ax.Box='off';
    grid(ax,'on'); ax.GridAlpha=.13;
    if ~isempty(speeds), xticks(ax,speeds); xlim(ax,[min(speeds)-50 max(speeds)+50]); end
end
end
function exportFigure(f,outDir,name)
exportgraphics(f,fullfile(outDir,[name '.png']),'Resolution',180);
exportgraphics(f,fullfile(outDir,[name '.pdf']),'ContentType','vector');
savefig(f,fullfile(outDir,[name '.fig'])); close(f);
end
function writeReport(outDir,dataRoot,T,S,R,withinR,B)
fid=fopen(fullfile(outDir,'分析说明.md'),'w','n','UTF-8');
assert(fid>=0,'Cannot write report.'); cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'# MATLAB 角速度与叩击加速度变异性分析\n\n');
fprintf(fid,'数据索引：`%s/groups.json`。共%d次FIRE，正常结束%d次，保护中止%d次。\n\n', ...
    dataRoot,height(T),sum(T.completed),sum(~T.completed));
fprintf(fid,'使用最终合并实验：300档18+2次，500档3+17次；600档采用重新开始的20次。早期联调及旧600档未混入。每条记录保留result_file、session、shot与档内顺序，MATLAB重新读取逐击原始CSV核对峰值和速度。\n\n');
fprintf(fid,'角速度为编码器轨迹既有速降前cutoff之前最后一个速度样本，非撤驱动时已速降的release_speed，也不是加速度峰值瞬间角速度。500档第3次保护中止，无有效角速度估计，因此该档角速度n=19；加速度可计算n=20，但该击窗口不完整。\n\n');
fprintf(fid,'加速度直接逐点计算sqrt(x^2+y^2+z^2)，再取FIRE后整个记录窗口内的最大值，不减任何基线；不平滑、不插值，单位counts，未换算g或力。此算法保留现行整动作窗口定义，启动振动或其他事件可能贡献最大峰值，未独立确认每个最大值都是接触峰值。约1kHz采样对窄冲击的峰值估计有限。\n\n');
fprintf(fid,'SD为样本标准差（n-1），CV=100×SD/abs(mean)。CV衡量重复性，目标偏差衡量准确性，分别报告；不跨速度档混算CV。缺失值不补零；n<2时SD/CV为空。\n\n');
fprintf(fid,'|目标(°/s)|速度n|速度均值±SD(°/s)|速度CV(%%)|目标偏差(%%)|加速度n|峰值均值±SD(counts)|峰值CV(%%)|组内r|\n');
fprintf(fid,'|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n');
A=S(S.subset=="all_recorded",:); C=R(R.subset=="all_recorded",:);
for i=1:height(A)
    a=A(i,:);
    fprintf(fid,'|%d|%d|%.2f ± %.2f|%.2f|%.2f|%d|%.2f ± %.2f|%.2f|%.3f|\n', ...
        a.target_dps,a.n_speed,a.speed_mean_dps,a.speed_sd_dps,a.speed_cv_pct, ...
        a.speed_bias_pct,a.n_accel,a.accel_mean_counts,a.accel_sd_counts,a.accel_cv_pct,C.pearson_r(i));
end
fprintf(fid,'\n角速度CV为%.2f%%—%.2f%%，记录到的加速度峰值CV为%.2f%%—%.2f%%；600°/s档加速度峰值CV最大（%.2f%%），而同档速度CV为%.2f%%。这说明当前采集链下两项重复性差异明显，不能仅凭速度重复性推定冲击峰值重复性。\n\n', ...
    min(A.speed_cv_pct),max(A.speed_cv_pct),min(A.accel_cv_pct),max(A.accel_cv_pct), ...
    A.accel_cv_pct(A.target_dps==600),A.speed_cv_pct(A.target_dps==600));
fprintf(fid,'新增独立的ADC顶码诊断：对FIRE后的原始轴值检查x/y/z >=4094或<=-4096。当前驱动右对齐、1600Hz输出；ADXL375在该ODR下最低位恒为0，因此正端可见顶码为4094。重复命中同一顶码与削顶一致，属于量程饱和证据；near_rail另外标记abs(axis)>=4000的记录，仅表示接近边界。参考[ADXL375官方数据手册，第24、28页](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl375.pdf)。\n\n');
fprintf(fid,'|目标(°/s)|顶码击次/总击次|接近边界击次|\n|---:|---:|---:|\n');
for i=1:height(A)
    fprintf(fid,'|%d|%d/%d|%d|\n',A.target_dps(i),A.n_rail_hit(i),A.n_records(i),A.n_near_rail(i));
end
fprintf(fid,'\n700/800/900°/s分别有15/17/15击达到顶码；这几档记录峰值与CV受削顶影响，800到900档峰值均值接近不能证明真实冲击强度进入平台，也不能把高档位CV下降解释为真实冲击更稳定。600档另有1击接近边界（Z轴−4088），虽未命中顶码仍需留意。这里只描述当前记录，不能恢复被削去的真实峰值。\n\n');
fprintf(fid,'\n全体可计算记录为主分析；normal_complete只保留正常结束且加速度窗口完整的记录；quality_qualified沿用采集时已有的复合质量标签作敏感性分析，未重新运行事件判定，当前峰值不使用该事件判定的基线，包含速度、事件数、完整性和链路条件，不能当作独立的加速度质量筛选。300档20次全部被标为多事件，复合质量子集为空，不代表20次都未叩击或存在二次接触。\n\n');
fprintf(fid,'complete_no_rail另外排除顶码记录，保留该选择后的n并仅作敏感性描述；排除主要发生于高冲击记录，会引入选择偏倚，不能用此子集代替真实高档位总体分布。原quality_qualified不含顶码判断，本分析保留其原值并另列rail_hit字段。\n\n');
fprintf(fid,'CV区间使用固定随机种子的%d次逐击自助重采样百分位区间，条件是假定击次独立；连续采集、会话切换和顺序效应可能使区间偏窄。图中误差条标清为SD或CV区间，不混为均值置信区间。不据此作显著性或因果判断。\n\n',B);
fprintf(fid,'各档内分别计算速度-加速度Pearson r；对两变量在各档内分别去均值并除以该档配对均值后合并，r=%.3f。该关联去除了档位均值差，尚未去除顺序和会话效应。跨档均值同向增长不能解释为单次速度波动导致峰值波动。\n\n',withinR);
fprintf(fid,'输出：01总体均值/SD/CV/目标偏差；02子集和CV区间；03逐击顺序；04配对关系；05角速度波形；06加速度峰值对齐波形；07顶码诊断。每图提供PNG、矢量PDF和可编辑FIG；analysis.mat包含逐击表、统计表和波形。两种波形使用不同时间原点，不用于精确推断冲击延迟。\n\n');
fprintf(fid,'复现：在仓库根目录运行 `addpath(''scripts''); analyze_hammer_variability;`。仅需基础MATLAB。版本：%s。\n',version);
end

function path=resolveArchivePath(root,path)
% Consolidated indexes use portable paths relative to the archive root.
% Existing absolute indexes remain supported for older data.
path=string(path);
if ~startsWith(path,"/") && isempty(regexp(char(path),'^[A-Za-z]:[\\/]','once'))
    path=string(fullfile(root,path));
end
end
