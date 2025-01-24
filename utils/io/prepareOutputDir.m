function output_dir = prepareOutputDir()
    output_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'outputs', 'plots');
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end
end
