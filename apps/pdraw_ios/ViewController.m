//
//  ViewController.m
//  PDrAW
//
//  Created by Aurelien Barre on 19/11/2017.
//  Copyright © 2017 Aurelien Barre. All rights reserved.
//

#import "ViewController.h"
#import <pdraw/pdraw.h>

@interface ViewController () {
    int _screenWidth;
    int _screenHeight;
    float _contentScaleFactor;
    uint64_t _lastRenderTime;
}

@property (strong, nonatomic) EAGLContext *context;
@property (nonatomic, assign) struct pdraw *pdraw;

- (void)setupPdraw;

@end

@implementation ViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    _lastRenderTime = 0;
    
    self.context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    
    if (!self.context) {
        NSLog(@"Failed to create ES context");
    }
    
    GLKView *view = (GLKView *)self.view;
    view.context = self.context;
    view.drawableDepthFormat = GLKViewDrawableDepthFormat24;
    self.preferredFramesPerSecond = 60;

    _screenWidth = [UIScreen mainScreen].bounds.size.width;
    _screenHeight = [UIScreen mainScreen].bounds.size.height;
    _contentScaleFactor = [UIScreen mainScreen].scale;

    [EAGLContext setCurrentContext:self.context];
	[self setupPdraw];
}

- (void)viewDidUnload
{
    [super viewDidUnload];

    if (self.pdraw) {
        int ret = pdraw_destroy(self.pdraw);
        if (ret != 0) {
            NSLog(@"pdraw_destroy() failed (%d)", ret);
        }
        self.pdraw = NULL;
    }
}

- (void)dealloc
{
    if ([EAGLContext currentContext] == self.context) {
        [EAGLContext setCurrentContext:nil];
    }
}

- (void)setupPdraw
{
    int ret;

    self.pdraw = pdraw_new();
    if (!self.pdraw) {
        NSLog(@"Failed to create PDrAW instance");
        return;
    }

    ret = pdraw_set_self_friendly_name(self.pdraw, "PDrAW-iOS"); //TODO
    if (ret != 0) {
        NSLog(@"pdraw_set_self_friendly_name() failed (%d)", ret);
    }

    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];
    NSString *fileName = [documentsDirectory stringByAppendingPathComponent:@"/drone.mp4"];
    BOOL fileExists = [[NSFileManager defaultManager] fileExistsAtPath:fileName];
    if (fileExists) {
        int ret = pdraw_open_url(self.pdraw, [fileName UTF8String]);
        if (ret != 0) {
            NSLog(@"pdraw_open_url() failed (%d)", ret);
        }
    } else {
        NSLog(@"File '%s' not found", [fileName UTF8String]);
        return;
    }

    ret = pdraw_start_renderer(self.pdraw,
                               (int)(_screenWidth * _contentScaleFactor),
                               (int)(_screenHeight * _contentScaleFactor), 0, 0,
                               (int)(_screenWidth * _contentScaleFactor),
                               (int)(_screenHeight * _contentScaleFactor),
                               0, 0, NULL);
    if (ret < 0) {
        NSLog(@"pdraw_start_renderer() failed (%d)", ret);
        return;
    }

    ret = pdraw_play(self.pdraw);
    if (ret != 0) {
        NSLog(@"pdraw_play() failed (%d)", ret);
        return;
    }
}

#pragma mark - GLKView and GLKViewController delegate methods

- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    if (self.pdraw) {
        int screenWidth = [UIScreen mainScreen].bounds.size.width;
        int screenHeight = [UIScreen mainScreen].bounds.size.height;
        float contentScaleFactor = [UIScreen mainScreen].scale;
        if ((screenWidth != _screenWidth) ||
            (screenHeight != _screenHeight) ||
            (contentScaleFactor != _contentScaleFactor)) {
            _screenWidth = screenWidth;
            _screenHeight = screenHeight;
            _contentScaleFactor = contentScaleFactor;
            int ret = pdraw_start_renderer(self.pdraw,
                                       (int)(_screenWidth * _contentScaleFactor),
                                       (int)(_screenHeight * _contentScaleFactor), 0, 0,
                                       (int)(_screenWidth * _contentScaleFactor),
                                       (int)(_screenHeight * _contentScaleFactor),
                                       0, 0, NULL);
            if (ret < 0) {
                NSLog(@"pdraw_start_renderer() failed (%d)", ret);
                return;
            }
        }

        struct timespec t1;
        int ret = pdraw_render(self.pdraw, _lastRenderTime);
        if (ret < 0) {
            NSLog(@"pdraw_render() failed (%d)", ret);
            return;
        }
        clock_gettime(CLOCK_MONOTONIC, &t1);
        _lastRenderTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
    }
}

@end
