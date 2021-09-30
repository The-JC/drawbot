function points = importsvg(filename)
% importsvg: converts SVG files to points
% Currently only supports a limited amount of SVG commands

points = [[], [], []];

if ~nargin
  return
end

% open file
fid = fopen(filename);

tline = fgetl(fid);
while ischar(tline)
    % line
    if contains(tline,'<line ')
        % Get index range of each tag  
        nameEnd = strfind(tline,'="')-1;
        nameStart = [strfind(tline,'<line ')+length('<line '), regexp(tline, '" |/')+2];
        % Get index range of each payload
        numStart = strfind(tline,'="')+2;
        numEnd = strfind(tline,'" ')-1;
        nameStart(end) = [];

        % Construct Vector 
        x1_ = [];
        x2_ = [];
        for i = 1:length(nameStart)
            if strcmp(tline(nameStart(i):nameEnd(i)), 'x1')
              x1_(1) = str2double(tline(numStart(i):numEnd(i)));
            end
            if strcmp(tline(nameStart(i):nameEnd(i)), 'y1')
              x1_(2) = str2double(tline(numStart(i):numEnd(i)));
            end
            if strcmp(tline(nameStart(i):nameEnd(i)), 'x2')
              x2_(1) = str2double(tline(numStart(i):numEnd(i)));
            end
            if strcmp(tline(nameStart(i):nameEnd(i)), 'y1')
              x2_(2) = str2double(tline(numStart(i):numEnd(i)));
            end
        end
        x1_(3) = 0;
        x2_(3) = 1;
        points = cat(1, points, [x1_; x2_]);
  % SVG Path
  elseif contains(tline,'<path ')
      % Get index range of each tag  
      nameEnd = strfind(tline,'="')-1;
      nameStart = [strfind(tline,'<path ')+length('<path '), regexp(tline, '" |/')+2];
      % Get index range of each payload  
      numStart = strfind(tline,'="')+2;
      numEnd = regexp(tline, '" |/')-1;
      nameStart(end) = [];
      for i = 1:length(nameStart)
        if strcmp(tline(nameStart(i):nameEnd(i)), 'd')
            d = tline(numStart(i):numEnd(i));
            d = replace(d, '"', '');
            p = svgPathParse(d);
            
            points = cat(1, points, p);
        end
      end
  end
  tline = fgetl(fid);
end

fclose(fid);

end
