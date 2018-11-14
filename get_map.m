% clc
% clear all
% close all

figure;
xlim([0,5580]);
ylim([0,1500]);
grid on;

rectangle('Position',[0,0,2180,80],'FaceColor',[0 0 0],'EdgeColor','k');
rectangle('Position',[2480,0,500,80],'FaceColor',[0 0 0],'EdgeColor','k');
rectangle('Position',[3500,0,2080,80],'FaceColor',[0 0 0],'EdgeColor','k');

rectangle('Position',[0,0,80,460],'FaceColor',[0 0 0],'EdgeColor','k');
rectangle('Position',[0,660,80,840],'FaceColor',[0 0 0],'EdgeColor','k');
rectangle('Position',[0,1420,1980,80],'FaceColor',[0 0 0],'EdgeColor','k');

rectangle('Position',[280,180,1700,280],'FaceColor',[0 0 0],'EdgeColor','k');
rectangle('Position',[280,660,1700,180],'FaceColor',[0 0 0],'EdgeColor','k');
rectangle('Position',[280,1040,1700,180],'FaceColor',[0 0 0],'EdgeColor','k');

rectangle('Position',[2180,660,1700,180],'FaceColor',[0 0 0],'EdgeColor','k');
rectangle('Position',[2180,1040,1700,180],'FaceColor',[0 0 0],'EdgeColor','k');
rectangle('Position',[2180,1420,3400,80],'FaceColor',[0 0 0],'EdgeColor','k');

rectangle('Position',[2180,280,500,180],'FaceColor',[0 0 0],'EdgeColor','k');
rectangle('Position',[2680,280,320,100],'FaceColor',[0 0 0],'EdgeColor','k');
rectangle('Position',[3200,280,200,100],'FaceColor',[0 0 0],'EdgeColor','k');

rectangle('Position',[5490,0,90,840],'FaceColor',[0 0 0],'EdgeColor','k');
rectangle('Position',[5490,960,90,540],'FaceColor',[0 0 0],'EdgeColor','k');

% %One way to left
% one_way_l = [280,80,1700,100];
% rectangle('Position',[280,80,1700,100],'FaceColor',[0 0.3 0.7],'EdgeColor','r');

% Initial state
rectangle('Position',[2050,1400,85,80],'FaceColor',[0 0.7 0.5],'EdgeColor','r');

% Goal 1
rectangle('Position',[3080,280,100,100],'FaceColor',[1 1 0],'EdgeColor','r');

% Goal 2
rectangle('Position',[5490,840,100,120],'FaceColor',[1 1 0],'EdgeColor','r');
