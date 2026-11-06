function ExportScheme2PointFiles(t, q, cartFromFk, cfg)
% ExportScheme2PointFiles
% 方案2只导出关节空间 PPB 点位文件。

if ~exist(cfg.outputDir, 'dir')
    mkdir(cfg.outputDir);
end

jointFile = fullfile(cfg.outputDir, 'joint.txt');

fid = fopen(jointFile, 'w');
for i = 1:cfg.exportEvery:size(q,1)
    fprintf(fid, '%.6f %.6f %.6f %.6f %.6f %.6f\n', rad2deg(q(i,:)));
end
fclose(fid);

end
