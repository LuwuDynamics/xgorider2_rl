"""MuJoCo rendering with Rider-owned input bindings, without simulate UI hotkeys.

Run on the normal Python main thread on macOS; this window owns the GLFW loop.
Only camera mouse gestures and explicitly registered robot keys are handled.
"""
import glfw
import mujoco
import numpy as np


class RiderWindow:
    def __init__(self,model,data,on_key):
        self.model=model;self.data=data
        if not glfw.init():raise RuntimeError('Cannot initialize the Rider window')
        self.window=glfw.create_window(1280,850,'Rider2 | MuJoCo | keyboard control',None,None)
        if not self.window:
            glfw.terminate();raise RuntimeError('Cannot create the Rider window')
        glfw.make_context_current(self.window);glfw.swap_interval(1)
        self.cam=mujoco.MjvCamera();self.cam.distance=.55;self.cam.azimuth=135;self.cam.elevation=-20
        self.opt=mujoco.MjvOption();self.scene=mujoco.MjvScene(model,maxgeom=10000)
        self.context=mujoco.MjrContext(model,mujoco.mjtFontScale.mjFONTSCALE_150.value)
        self.camera_offset=np.zeros(3);self.cursor=glfw.get_cursor_pos(self.window)

        def key(window,code,scancode,action,modifiers):
            if action!=glfw.PRESS:return
            if code==glfw.KEY_ESCAPE:glfw.set_window_should_close(window,True)
            elif code==glfw.KEY_F:self.camera_offset[:]=0
            else:on_key(code)

        def button(window,code,action,modifiers):
            self.cursor=glfw.get_cursor_pos(window)

        def cursor(window,x,y):
            dx=x-self.cursor[0];dy=y-self.cursor[1];self.cursor=(x,y)
            left=glfw.get_mouse_button(window,glfw.MOUSE_BUTTON_LEFT)==glfw.PRESS
            right=glfw.get_mouse_button(window,glfw.MOUSE_BUTTON_RIGHT)==glfw.PRESS
            middle=glfw.get_mouse_button(window,glfw.MOUSE_BUTTON_MIDDLE)==glfw.PRESS
            if not (left or right or middle):return
            shift=any(glfw.get_key(window,key)==glfw.PRESS for key in (glfw.KEY_LEFT_SHIFT,glfw.KEY_RIGHT_SHIFT))
            if right:action=mujoco.mjtMouse.mjMOUSE_MOVE_H if shift else mujoco.mjtMouse.mjMOUSE_MOVE_V
            elif left:action=mujoco.mjtMouse.mjMOUSE_ROTATE_H if shift else mujoco.mjtMouse.mjMOUSE_ROTATE_V
            else:action=mujoco.mjtMouse.mjMOUSE_ZOOM
            height=max(1,glfw.get_window_size(window)[1]);before=self.cam.lookat.copy()
            mujoco.mjv_moveCamera(model,action,dx/height,dy/height,self.cam)
            self.camera_offset+=self.cam.lookat-before

        def scroll(window,x,y):
            mujoco.mjv_moveCamera(model,mujoco.mjtMouse.mjMOUSE_ZOOM,0,-.05*y,self.cam)

        self.callbacks=(key,button,cursor,scroll)
        glfw.set_key_callback(self.window,key);glfw.set_mouse_button_callback(self.window,button)
        glfw.set_cursor_pos_callback(self.window,cursor);glfw.set_scroll_callback(self.window,scroll)

    def is_running(self):return not glfw.window_should_close(self.window)

    def poll(self):glfw.poll_events()

    def render(self,target,left_text,right_text):
        width,height=glfw.get_framebuffer_size(self.window)
        if width<=0 or height<=0:return
        self.cam.lookat[:]=target+self.camera_offset
        mujoco.mjv_updateScene(self.model,self.data,self.opt,None,self.cam,mujoco.mjtCatBit.mjCAT_ALL,self.scene)
        viewport=mujoco.MjrRect(0,0,width,height)
        mujoco.mjr_render(viewport,self.scene,self.context)
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,
            viewport,left_text,right_text,self.context)
        glfw.swap_buffers(self.window)

    def close(self):
        if self.context is not None:self.context.free();self.context=None
        self.scene=None  # MjvScene is released by its Python binding destructor.
        if self.window is not None:glfw.destroy_window(self.window);self.window=None
        glfw.terminate()

    def __enter__(self):return self
    def __exit__(self,*args):self.close()
