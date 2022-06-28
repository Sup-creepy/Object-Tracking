function varargout = object_tracking_gui(varargin)
% OBJECT_TRACKING_GUI MATLAB code for object_tracking_gui.fig
%      OBJECT_TRACKING_GUI, by itself, creates a new OBJECT_TRACKING_GUI or raises the existing
%      singleton*.
%
%      H = OBJECT_TRACKING_GUI returns the handle to a new OBJECT_TRACKING_GUI or the handle to
%      the existing singleton*.
%
%      OBJECT_TRACKING_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in OBJECT_TRACKING_GUI.M with the given input arguments.
%
%      OBJECT_TRACKING_GUI('Property','Value',...) creates a new OBJECT_TRACKING_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before object_tracking_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to object_tracking_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help object_tracking_gui

% Last Modified by GUIDE v2.5 25-Jun-2022 16:02:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @object_tracking_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @object_tracking_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
end

% --- Executes just before object_tracking_gui is made visible.
function object_tracking_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to object_tracking_gui (see VARARGIN)

% Choose default command line output for object_tracking_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes object_tracking_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);

end

% --- Outputs from this function are returned to the command line.
function varargout = object_tracking_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

end

function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
end

% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end

% 打开文件按钮
function openfile_Callback(hObject, eventdata, handles)
% hObject    handle to openfile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uigetfile({'*.avi'; '*.mp4'},'打开视频');
str = [pathname filename];
set(handles.edit1, 'String', str);
end

% 退出按钮
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(gcf);
global  exit_flag ;
exit_flag = true;
end

% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1
end

% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes3
end

%************************************************************************%

%% 开始按钮,主程序
function start_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global exit_flag;
global pause_flag;
exit_flag = false;


videoName = get(handles.edit1, 'String');
videoSource = vision.VideoFileReader(videoName,...
    'ImageColorSpace', 'RGB', 'VideoOutputDataType', 'uint8');   %读取视频




% 创建用于前景检测和分析的系统对象
% 前景检测器用于从背景分割运动对象。它输出一个二进制代码，其中像素值1对应于前景，而值0对应于背景。
%设置检测子为混合高斯模型，高斯核数目：3，训练背景帧数：40，背景阈值：0.7
detector = vision.ForegroundDetector('NumGaussians', 3, ...
    'NumTrainingFrames', 30, 'MinimumBackgroundRatio', 0.7);

%设置blob分析子，寻找联通域，得到连通区域的几何特征
blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
    'AreaOutputPort', true, 'CentroidOutputPort', true, ...
    'MinimumBlobArea', 100);    %最小区域面积：400；
%%  初始化轨迹
tracks = struct(...
        'id', {}, ...                       %轨迹ID
        'bbox', {}, ...                     %对象的当前边界框；用于显示
        'kalmanFilter', {}, ...             %轨迹的卡尔曼滤波器
        'age', {}, ...                      %自首次检测到轨迹以来的帧数
        'totalVisibleCount', {}, ...        %可见总帧数
        'consecutiveInvisibleCount', {});   %连续不可见帧数
nextId = 1;                     % 下一个轨迹的ID
current_frame = 0;

%%  主程序  循环检测帧并显示跟踪结果
while ~isDone(videoSource) && ~exit_flag      
    frame = videoSource();              %读取视频帧
    mask = step(detector,frame);        %使用检测子（混合高斯模型）得到前景图

    % 对生成的二进制蒙版执行形态学操作：腐蚀和膨胀，最后补洞，消除物体中间的空洞
    mask = imopen(mask, strel('rectangle', [8,8]));     %开运算
    mask = imclose(mask, strel('rectangle', [15, 15])); %闭运算
    mask = imfill(mask, 'holes');

    % 使用blob分析得到的所有连通域的中心，跟踪矩形框大小
    [~, centroids, bboxes] = step(blobAnalyser,mask);
    
    %% 预测以跟踪轨迹的新位置
    for i = 1:length(tracks)
        bbox = tracks(i).bbox;

        % 使用卡尔曼滤波器，根据以前的轨迹，预测当前位置中心
        predictedCentroid = predict(tracks(i).kalmanFilter);

        % 调整好预测中心位置后，将结果作为轨迹的跟踪矩形框
        predictedCentroid = int32(predictedCentroid) - bbox(3:4) / 2;
        tracks(i).bbox = [predictedCentroid, bbox(3:4)];    %当前位置真实值
    end
    
    %% 分配新检测目标给轨迹   
    %获取轨迹个数和新检测目标的个数
    nTracks = length(tracks);
    nDetections = size(centroids, 1);

    %创建损失函数矩阵，行代表轨迹，列代表新检测目标
    cost = zeros(nTracks, nDetections);
    % 对每个轨迹来说，使用他们的卡尔曼滤波器预测的结果，与每个新检测目标的中心计算欧氏距离，
    % 存入损失函数矩阵中

    for i = 1:nTracks
        cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
    end

    % 设置阈值为20，当根据算法得到的分数低于20时就不分配
    costOfNonAssignment = 20;
    [assignments, unassignedTracks, unassignedDetections] = ...
        assignDetectionsToTracks(cost, costOfNonAssignment);
    
    %%  更新已分配的轨迹
    numAssignedTracks = size(assignments, 1);
    for i = 1:numAssignedTracks
        trackIdx = assignments(i, 1);
        detectionIdx = assignments(i, 2);
        centroid = centroids(detectionIdx, :);
        bbox = bboxes(detectionIdx, :);

        % 根据轨迹对应的检测目标位置中心修正他的卡尔曼滤波器
        correct(tracks(trackIdx).kalmanFilter, centroid);

        % 用检测到的边界框替换预测的边界框
        tracks(trackIdx).bbox = bbox;
        
        % 轨迹数量+1
        tracks(trackIdx).age = tracks(trackIdx).age + 1;

        % 轨迹总可见帧数+1，轨迹连续不可见帧数清零
        tracks(trackIdx).totalVisibleCount = ...
            tracks(trackIdx).totalVisibleCount + 1;
        tracks(trackIdx).consecutiveInvisibleCount = 0;
    end
    
    %% 更新未分配的轨迹
    for i = 1:length(unassignedTracks)
            ind = unassignedTracks(i);
            tracks(ind).age = tracks(ind).age + 1;      %轨迹数量+1
            tracks(ind).consecutiveInvisibleCount = ...
            tracks(ind).consecutiveInvisibleCount + 1;  %轨迹总可见帧数+1
    end
    
    %%  删除丢失的轨迹
    if ~isempty(tracks)
        % 设置阈值为20，意思是当前连续不可见帧数大于等于20时就丢弃轨迹
        invisibleForTooLong = 20;
        % 设置阈值为8，指的是当年龄小于8时，根据总可见的帧数与年龄的比值
        ageThreshold = 8;

        % 计算轨道可见的年龄百分比
        ages = [tracks(:).age];
        totalVisibleCounts = [tracks(:).totalVisibleCount];
        visibility = totalVisibleCounts ./ ages;

        %（该比值阈值设为0.6）丢弃轨迹。否则根据连续不可见帧数丢弃轨迹
        lostInds = (ages < ageThreshold & visibility < 0.6) | ...
            [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;

        % 删除丢失轨迹
        tracks = tracks(~lostInds);
    end
    
    
    %%  创建新轨迹
    centroids = centroids(unassignedDetections, :);
    bboxes = bboxes(unassignedDetections, :);
    for i = 1:size(centroids, 1)
        centroid = centroids(i,:);
        bbox = bboxes(i, :);
        
        % 创建卡尔曼滤波器
        % 动态模型：匀速 
        % 初始化位置：新检测目标中心
        % 初始化估计误差（位置误差、速度误差）：[200，50]
        % 动态噪声（位置误差，速度误差）：[100，25]
        % 测量噪声（位置噪声）：100
        kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
            centroid, [200, 50], [100, 25], 100);

        % 创建新轨迹
        newTrack = struct(...
            'id', nextId, ...
            'bbox', bbox, ...
            'kalmanFilter', kalmanFilter, ...
            'age', 1, ...
            'totalVisibleCount', 1, ...
            'consecutiveInvisibleCount', 0);
        tracks(end + 1) = newTrack;
        nextId = nextId + 1;    %增加新ID
    end
    
    %%  显示跟踪结果
    % Convert the frame and the mask to uint8 RGB.
    frame = im2uint8(frame);
    mask = uint8(repmat(mask, [1, 1, 3])) .* 255;

    minVisibleCount = 8;
    if ~isempty(tracks)

        reliableTrackInds = ...
            [tracks(:).totalVisibleCount] > minVisibleCount;
        reliableTracks = tracks(reliableTrackInds);
        % 当总可见帧数大于阈值minVisibleCount=8时才显示出来，防止一些噪声产生

        % 播放视频，如果目标在该帧下未被检测，则显示预测矩形框
        if ~isempty(reliableTracks)
            % 获取边界框
            bboxes = cat(1, reliableTracks.bbox);

            % 获取ID
            ids = int32([reliableTracks(:).id]);
            n = length(ids);    %当前帧下目标的个数
            % 为目标预测的位置创建标签
            labels = cellstr(int2str(ids'));
            predictedTrackInds = ...
                [reliableTracks(:).consecutiveInvisibleCount] > 0;
            isPredicted = cell(size(labels));
            isPredicted(predictedTrackInds) = {' predicted'};
            labels = strcat(labels, isPredicted);
            
            current_frame = current_frame+1;
            for i = 1:n
                px(i,current_frame)=round(bboxes(i,1)+bboxes(i,3)/2);%找到质心的x值
                py(i,current_frame)=round(bboxes(i,2)+bboxes(i,4)/2);%找到质心的y值
            end

            % 显示目标矩形框
            frame = insertObjectAnnotation(frame, 'rectangle',bboxes, labels);

            % 显示前景图中的矩形框
            mask = insertObjectAnnotation(mask, 'rectangle',bboxes, labels);

        end
    end 
    axes(handles.axes1);    
    imshow(frame);
    hold on;
    if(current_frame > 40)  %当帧数大于40时显示运动轨迹
        for k = 1:n
        scatter(px(k,current_frame-40:current_frame),py(k,current_frame-40:current_frame),10,...
            'MarkerEdgeColor','none','MarkerFaceColor',[1 0 0],'LineWidth',0.1);
        end
    end
    hold off;

    axes(handles.axes3);
    imshow(mask);

    
end


end
