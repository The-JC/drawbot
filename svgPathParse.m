function points = svgPathParse(pathstring)
% svgPathParse convert a SVG path to points to be drawn

% Regular expressions for parsing SVG paths via first splitting into command
% groups, and then splitting each group into numeric arguments.
cmdsplit = '\s*([mMzZlLhHvVcCsSqQtTaA])\s*';
numsplit = [ ...
    '\s*,\s*|'                ... % Split at comma with whitespace, or
    '\s+|'                    ... % split at whitespace, or
    '(?<=[0-9])(?=[-+])|'     ... % split before a sign, or
    '(?<=[.eE][0-9]+)(?=[.])' ... % split before a second decimal point.
    ];

[cmds, allparams] = regexp(pathstring, cmdsplit, 'match', 'split');
cmds = strtrim(cmds);           % Trim any excess whitespace.
allparams = allparams(2:end);   % Ignore part before first command.
allparams = regexp(allparams, numsplit, 'split', 'emptymatch');

% Starting at the point (0, 0), keep a running tally of where the start point
% should be for the next segment. Loop through all of the command blocks,
% and for each one, loop through its constituent segments. For each one,
% add the appropriate command to segmentcmds, and the full set of
% control point locations including the starting point to segmentcoeffs.
% Also keep a running tally of the number of segments, starting at zero.

points = [[], [], []];
startpoint = [0 0];
segmentcmds = {};
segmentcoeffs = {};
segments = 0;
for cmdIdx=1:numel(cmds)
    params = str2double(allparams{cmdIdx});
    nparams = numel(params);
    switch cmds{cmdIdx}
        case 'm'  % Move to:
            endpoint = startpoint + params(1:2);
            p = toPoint(endpoint, 0);
            points = cat(1, points, p);
            startpoint = endpoint;
        case 'M'
            endpoint = params(1:2);
            p = toPoint(endpoint, 0);
            points = cat(1, points, p);
            startpoint = endpoint;
        case 'l'  % Line to:
            for k=1:2:nparams
                endpoint = startpoint + params(k:k+1);
                p = toPoint(endpoint, 1);
                points = cat(1, points, p);
                startpoint = endpoint;
            end
        case 'L'
            for k=1:2:nparams
                endpoint = params(k:k+1);
                p = toPoint(endpoint, 1);
                points = cat(1, points, p);
                startpoint = endpoint;
            end
        case 'h'  % Horizontal line to:
            for k=1:nparams
                endpoint = startpoint + [params(k) 0];
                p = toPoint(endpoint, 1);
                points = cat(1, points, p);
                startpoint = endpoint;
            end
        case 'H'
            for k=1:nparams
                endpoint = [params(k) startpoint(2)];
                p = toPoint(endpoint, 1);
                points = cat(1, points, p);
                startpoint = endpoint;
            end
        case 'v'  % Vertical line to:
            for k=1:nparams
                endpoint = startpoint + [0 params(k)];
                p = toPoint(endpoint, 1);
                points = cat(1, points, p);
                startpoint = endpoint;
            end
        case 'V'
            for k=1:nparams
                endpoint = [startpoint(2) params(k)];
                p = toPoint(endpoint, 1);
                points = cat(1, points, p);
                startpoint = endpoint;
            end
        case 'c'  % Cubic curve to:
            for k=1:6:nparams
                ctrlpt1 = startpoint + params(k:k+1);
                ctrlpt2 = startpoint + params(k+2:k+3);
                endpoint = startpoint + params(k+4:k+5);
                
                p = cubicBezier(startpoint, ctrlpt1, ctrlpt2, endpoint);
                points = cat(1, points, p);
                startpoint = endpoint;
            end
        case 'C'
            for k=1:6:nparams
                ctrlpt1 = params(k:k+1);
                ctrlpt2 = params(k+2:k+3);
                endpoint = params(k+4:k+5);
                
                p = cubicBezier(startpoint, ctrlpt1, ctrlpt2, endpoint);
                points = cat(1, points, p);
                startpoint = endpoint;
            end
        case 's'  % Smooth cubic to:
            for k=1:4:nparams
%                 ctrlpt1 = startpoint + params(k:k+1);
%                 ctrlpt2 = startpoint + params(k+2:k+3);
%                 endpoint = startpoint + params(k+4:k+5);
%                 
%                 p = cubicBezier(startpoint, ctrlpt1, ctrlpt2, endpoint);
%                 points = cat(1, points, p);
%                 startpoint = endpoint;
                
%                 segments = segments + 1;
%                 segmentcmds{segments} = 'C';
%                 if strcmp(segmentcmds{segments-1}, 'C');
%                     prevctrlpt2 = segmentcoeffs{segments-1}(3,:);
%                     ctrlpt1 = 2 * startpoint - prevctrlpt2;
%                 else
%                     ctrlpt1 = startpoint;
%                 end
%                 ctrlpt2 = startpoint + params(k:k+1);
%                 endpoint = startpoint + params(k+2:k+3);
%                 segmentcoeffs{segments} = [startpoint; ctrlpt1; ctrlpt2; endpoint];
%                 startpoint = endpoint;
            end
        case 'S'
            for k=1:4:nparams
%                 ctrlpt1 = params(k:k+1);
%                 ctrlpt2 = params(k+2:k+3);
%                 endpoint = params(k+4:k+5);
%                 
%                 p = cubicBezier(startpoint, ctrlpt1, ctrlpt2, endpoint);
%                 points = cat(1, points, p);
%                 startpoint = endpoint;
%                 segments = segments + 1;
%                 segmentcmds{segments} = 'C';
%                 if strcmp(segmentcmds{segments-1}, 'C');
%                     prevctrlpt2 = segmentcoeffs{segments-1}(3,:);
%                     ctrlpt1 = 2 * startpoint - prevctrlpt2;
%                 else
%                     ctrlpt1 = startpoint;
%                 end
%                 ctrlpt2 = params(k:k+1);
%                 endpoint = params(k+2:k+3);
%                 segmentcoeffs{segments} = [startpoint; ctrlpt1; ctrlpt2; endpoint];
%                 startpoint = endpoint;
            end
        case 'q'  % Quadratic curve to:
            for k=1:4:nparams
                ctrlpt = startpoint + params(k:k+1);
                endpoint = startpoint + params(k+2:k+3);
                
                p = quadBezier(startpoint, ctrlpt, endpoint);
                points = cat(1, points, p);
                startpoint = endpoint;
            end
        case 'Q'
            for k=1:4:nparams
                ctrlpt = params(k:k+1);
                endpoint = params(k+2:k+3);
                
                p = quadBezier(startpoint, ctrlpt, endpoint);
                points = cat(1, points, p);
                startpoint = endpoint;
            end
        otherwise
%         case 't'  % Smooth quadratic to:
%             for k=1:2:nparams
%                 segments = segments + 1;
%                 segmentcmds{segments} = 'Q';
%                 if strcmp(segmentcmds{segments-1}, 'Q');
%                     prevctrlpt = segmentcoeffs{segments-1}(2,:);
%                     ctrlpt = 2 * startpoint - prevctrlpt;
%                 else
%                     ctrlpt = startpoint;
%                 end
%                 endpoint = startpoint + params(k:k+1);
%                 segmentcoeffs{segments} = [startpoint; ctrlpt; endpoint];
%                 startpoint = endpoint;
%             end
%         case 'T'
%             for k=1:2:nparams
%                 segments = segments + 1;
%                 segmentcmds{segments} = 'Q';
%                 if strcmp(segmentcmds{segments-1}, 'Q');
%                     prevctrlpt = segmentcoeffs{segments-1}(2,:);
%                     ctrlpt = 2 * startpoint - prevctrlpt;
%                 else
%                     ctrlpt = startpoint;
%                 end
%                 endpoint = params(k:k+1);
%                 segmentcoeffs{segments} = [startpoint; ctrlpt; endpoint];
%                 startpoint = endpoint;
%             end
%         case 'a'  % Elliptical arc to:
%             % TODO: Implement elliptical arcs, cf.:
%             % http://www.w3.org/TR/SVG/implnote.html#ArcImplementationNotes
%             for k=1:7:nparams
%                 segments = segments + 1;
%                 segmentcmds{segments} = 'L';
%                 endpoint = startpoint + params(k+5:k+6);
%                 segmentcoeffs{segments} = [startpoint; endpoint];
%                 startpoint = endpoint;
%             end
%         case 'A'
%             for k=1:7:nparams
%                 segments = segments + 1;
%                 segmentcmds{segments} = 'L';
%                 endpoint = params(k+5:k+6);
%                 segmentcoeffs{segments} = [startpoint; endpoint];
%                 startpoint = endpoint;
%             end
%         case 'z'  % Close path:
%             % Do nothing in this case.
%         case 'Z'
    end
end


end

function points = cubicBezier(x1_, x2_, x3_, x4_)
    arguments
        x1_(1,2) double;
        x2_(1,2) double;
        x3_(1,2) double;
        x4_(1,2) double;
    end
    t = linspace(0, 1, 5)';
    
    pts = kron((1-t).^3,x1_) + kron(3*(1-t).^2.*t,x2_) + kron(3*(1-t).*t.^2,x3_) + kron(t.^3,x4_);
    
    points = [[], [],[]];
    for i = 1:length(pts)
        points(i,:) = toPoint(pts(i,:), 1);
    end
end

function points = quadBezier(x1_, x2_, x3_)
    arguments
        x1_(1,2) double;
        x2_(1,2) double;
        x3_(1,2) double;
    end
    t = linspace(0, 1, 5)';
    
    pts = kron((1-t).^2,x1_) + kron(2*(1-t).*t,x2_) + kron(t.^2,x3_);
    
    points = [[], [],[]];
    for i = 1:length(pts)
        points(i,:) = toPoint(pts(i,:), 1);
    end
end

function point = toPoint(x_, penDown)
    point = [x_(1), x_(2), penDown];
end

