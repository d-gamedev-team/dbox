/*
 * Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
module framework.main;

import std.algorithm;
import std.exception;
import std.file;
import std.path;
import std.stdio;
import std.string;

import deimos.glfw.glfw3;

import glad.gl.enums;
import glad.gl.ext;
import glad.gl.funcs;
import glad.gl.loader;
import glad.gl.types;

import glwtf.input;
import glwtf.window;

import dbox;

import imgui;
import imgui.engine;

import tests.test_entries;

import framework.debug_draw;
import framework.test;
import framework.window;

enum RED    = RGBA(255,   0,   0, 255);
enum GREEN  = RGBA(  0, 255,   0, 255);
enum BLUE   = RGBA(  0,   0, 255, 255);
enum WHITE  = RGBA(255, 255, 255, 255);
enum SILVER = RGBA(220, 220, 220, 255);

const entryTestName = "Cantilever";

//
struct UIState
{
    bool showMenu;
    int scroll;
    int scrollarea1;
    bool mouseOverMenu;
    bool chooseTest;
}

GLFWwindow* mainWindow;
UIState ui;

int32 testIndex;
int32 testSelection;
TestEntry* entry;
Test test;
Settings settings;
bool rightMouseDown;
b2Vec2 lastp;

//
void sCreateUI()
{
    ui.showMenu      = true;
    ui.scroll        = 0;
    ui.scrollarea1   = 0;
    ui.chooseTest    = false;
    ui.mouseOverMenu = false;

    string fontPath = thisExePath().dirName().buildPath("../examples/DroidSans.ttf");

    if (imguiInit(fontPath) == false)
    {
        fprintf(stderr.getFP(), "Could not init GUI renderer.\n");
        assert(false);
    }
}

//
extern(C) void sResizeWindow(GLFWwindow*, int width, int height)
{
    g_camera.m_width  = width;
    g_camera.m_height = height;
}

//
extern(C) void sKeyCallback(GLFWwindow*, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS)
    {
        switch (key)
        {
            case GLFW_KEY_ESCAPE:

                // Quit
                glfwSetWindowShouldClose(mainWindow, GL_TRUE);
                break;

            case GLFW_KEY_LEFT:

                // Pan left
                if (mods == GLFW_MOD_CONTROL)
                {
                    b2Vec2 newOrigin = b2Vec2(2.0f, 0.0f);
                    test.ShiftOrigin(newOrigin);
                }
                else
                {
                    g_camera.m_center.x -= 0.5f;
                }
                break;

            case GLFW_KEY_RIGHT:

                // Pan right
                if (mods == GLFW_MOD_CONTROL)
                {
                    b2Vec2 newOrigin = b2Vec2(-2.0f, 0.0f);
                    test.ShiftOrigin(newOrigin);
                }
                else
                {
                    g_camera.m_center.x += 0.5f;
                }
                break;

            case GLFW_KEY_DOWN:

                // Pan down
                if (mods == GLFW_MOD_CONTROL)
                {
                    b2Vec2 newOrigin = b2Vec2(0.0f, 2.0f);
                    test.ShiftOrigin(newOrigin);
                }
                else
                {
                    g_camera.m_center.y -= 0.5f;
                }
                break;

            case GLFW_KEY_UP:

                // Pan up
                if (mods == GLFW_MOD_CONTROL)
                {
                    b2Vec2 newOrigin = b2Vec2(0.0f, -2.0f);
                    test.ShiftOrigin(newOrigin);
                }
                else
                {
                    g_camera.m_center.y += 0.5f;
                }
                break;

            case GLFW_KEY_HOME:

                // Reset view
                g_camera.m_zoom = 1.0f;
                g_camera.m_center.Set(0.0f, 20.0f);
                break;

            case GLFW_KEY_Z:

                // Zoom out
                g_camera.m_zoom = b2Min(1.1f * g_camera.m_zoom, 20.0f);
                break;

            case GLFW_KEY_X:

                // Zoom in
                g_camera.m_zoom = b2Max(0.9f * g_camera.m_zoom, 0.02f);
                break;

            case GLFW_KEY_R:

                // Reset test

                test = entry.createFcn();
                break;

            case GLFW_KEY_SPACE:

                // Launch a bomb.
                if (test)
                {
                    test.LaunchBomb();
                }
                break;

            case GLFW_KEY_P:

                // Pause
                settings.pause = !settings.pause;
                break;

            case GLFW_KEY_LEFT_BRACKET:

                // Switch to previous test
                --testSelection;

                if (testSelection < 0)
                {
                    testSelection = g_testEntries.length - 1;
                }
                break;

            case GLFW_KEY_RIGHT_BRACKET:

                // Switch to next test
                ++testSelection;

                if (testSelection == g_testEntries.length)
                {
                    testSelection = 0;
                }
                break;

            case GLFW_KEY_TAB:
                ui.showMenu = !ui.showMenu;
                break;

            default:

                if (test)
                {
                    test.Keyboard(key);
                }
        }
    }
    else if (action == GLFW_RELEASE)
    {
        test.KeyboardUp(key);
    }

    // else GLFW_REPEAT
}

//
extern(C) void sMouseButton(GLFWwindow*, int32 button, int32 action, int32 mods)
{
    double xd, yd;
    glfwGetCursorPos(mainWindow, &xd, &yd);
    b2Vec2 ps = b2Vec2(cast(float32)xd, cast(float32)yd);

    // Use the mouse to move things around.
    if (button == GLFW_MOUSE_BUTTON_1)
    {
        // <##>
        // ps.Set(0, 0);
        b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);

        if (action == GLFW_PRESS)
        {
            if (mods == GLFW_MOD_SHIFT)
            {
                test.ShiftMouseDown(pw);
            }
            else
            {
                test.MouseDown(pw);
            }
        }

        if (action == GLFW_RELEASE)
        {
            test.MouseUp(pw);
        }
    }
    else if (button == GLFW_MOUSE_BUTTON_2)
    {
        if (action == GLFW_PRESS)
        {
            lastp = g_camera.ConvertScreenToWorld(ps);
            rightMouseDown = true;
        }

        if (action == GLFW_RELEASE)
        {
            rightMouseDown = false;
        }
    }
}

//
extern(C) void sMouseMotion(GLFWwindow*, double xd, double yd)
{
    b2Vec2 ps = b2Vec2(cast(float)xd, cast(float)yd);

    b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
    test.MouseMove(pw);

    if (rightMouseDown)
    {
        b2Vec2 diff = pw - lastp;
        g_camera.m_center.x -= diff.x;
        g_camera.m_center.y -= diff.y;
        lastp = g_camera.ConvertScreenToWorld(ps);
    }
}

//
extern(C) void sScrollCallback(GLFWwindow*, double, double dy)
{
    if (ui.mouseOverMenu)
    {
        ui.scroll = -cast(int)dy;
    }
    else
    {
        if (dy > 0)
        {
            g_camera.m_zoom /= 1.1f;
        }
        else
        {
            g_camera.m_zoom *= 1.1f;
        }
    }
}

//
void sRestart()
{
    entry = &g_testEntries[testIndex];
    test  = entry.createFcn();
}

//
void sSimulate()
{
    glEnable(GL_DEPTH_TEST);
    test.Step(&settings);

    test.DrawTitle(entry.name);
    glDisable(GL_DEPTH_TEST);

    if (testSelection != testIndex)
    {
        testIndex = testSelection;

        entry = &g_testEntries[testIndex];
        test  = entry.createFcn();
        g_camera.m_zoom = 1.0f;
        g_camera.m_center.Set(0.0f, 20.0f);
    }
}

//
void sInterface()
{
    int menuWidth = 200;
    ui.mouseOverMenu = false;

    if (ui.showMenu)
    {
        bool over = imguiBeginScrollArea("Testbed Controls", g_camera.m_width - menuWidth - 10, 10, menuWidth, g_camera.m_height - 20, &ui.scrollarea1);

        if (over)
            ui.mouseOverMenu = true;

        imguiSeparatorLine();

        imguiLabel("Test");

        if (imguiButton(entry.name, Enabled.yes))
        {
            ui.chooseTest = !ui.chooseTest;
        }

        imguiSeparatorLine();

        imguiSlider("Vel Iters", &settings.velocityIterations, 0, 50, 1, Enabled.yes);
        imguiSlider("Pos Iters", &settings.positionIterations, 0, 50, 1, Enabled.yes);
        imguiSlider("Hertz", &settings.hz, 5.0f, 120.0f, 5.0f, Enabled.yes);

        imguiCheck("Sleep", &settings.enableSleep, Enabled.yes);

        imguiCheck("Warm Starting", &settings.enableWarmStarting, Enabled.yes);

        imguiCheck("Time of Impact", &settings.enableContinuous, Enabled.yes);

        imguiCheck("Sub-Stepping", &settings.enableSubStepping, Enabled.yes);

        imguiSeparatorLine();

        imguiCheck("Shapes", &settings.drawShapes, Enabled.yes);

        imguiCheck("Joints", &settings.drawJoints, Enabled.yes);

        imguiCheck("AABBs", &settings.drawAABBs, Enabled.yes);

        imguiCheck("Contact Points", &settings.drawContactPoints, Enabled.yes);

        imguiCheck("Contact Normals", &settings.drawContactNormals, Enabled.yes);

        imguiCheck("Contact Impulses", &settings.drawContactImpulse, Enabled.yes);

        imguiCheck("Friction Impulses", &settings.drawFrictionImpulse, Enabled.yes);

        imguiCheck("Center of Masses", &settings.drawCOMs, Enabled.yes);

        imguiCheck("Statistics", &settings.drawStats, Enabled.yes);

        imguiCheck("Profile", &settings.drawProfile, Enabled.yes);

        if (imguiButton(settings.pause ? "Resume" : "Pause", Enabled.yes))
            settings.pause = !settings.pause;

        if (imguiButton("Single Step", Enabled.yes))
        {
            if (!settings.pause)
                settings.pause = true;

            settings.singleStep = !settings.singleStep;
        }

        if (imguiButton("Restart", Enabled.yes))
            sRestart();

        if (imguiButton("Quit", Enabled.yes))
            glfwSetWindowShouldClose(mainWindow, GL_TRUE);

        imguiEndScrollArea();
    }

    int testMenuWidth = 200;

    if (ui.chooseTest)
    {
        static int testScroll = 0;
        bool over = imguiBeginScrollArea("Choose Sample", g_camera.m_width - menuWidth - testMenuWidth - 20, 10, testMenuWidth, g_camera.m_height - 20, &testScroll);

        if (over)
            ui.mouseOverMenu = true;

        for (int i = 0; i < g_testEntries.length; ++i)
        {
            if (imguiItem(g_testEntries[i].name, Enabled.yes))
            {
                entry         = &g_testEntries[i];
                test          = entry.createFcn();
                ui.chooseTest = false;
            }
        }

        imguiEndScrollArea();
    }

    imguiEndFrame();
}

void runTests()
{
    g_debugDraw = new DebugDraw();
    g_camera.m_width  = winSize.width;
    g_camera.m_height = winSize.height;

    auto res = glfwInit();
    enforce(res, format("glfwInit call failed with return code: '%s'", res));
    scope(exit)
        glfwTerminate();

    char title[64];
    sprintf(title.ptr, "Box2D Testbed Version %d.%d.%d", b2_version.major, b2_version.minor, b2_version.revision);

    auto window = createWindow("imgui", WindowMode.windowed, winSize.width, winSize.height);
    mainWindow = window.window;

    if (mainWindow is null)
    {
        fprintf(stderr.getFP(), "Failed to open GLFW mainWindow.\n");
        glfwTerminate();
        assert(0);
    }

    glfwMakeContextCurrent(mainWindow);

    // Load all OpenGL function pointers via glad.
    enforce(gladLoadGL());

    // printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

    glfwSetScrollCallback(mainWindow, &sScrollCallback);
    glfwSetWindowSizeCallback(mainWindow, &sResizeWindow);
    glfwSetKeyCallback(mainWindow, &sKeyCallback);
    glfwSetMouseButtonCallback(mainWindow, &sMouseButton);
    glfwSetCursorPosCallback(mainWindow, &sMouseMotion);
    glfwSetScrollCallback(mainWindow, &sScrollCallback);

    g_debugDraw.Create();

    sCreateUI();

    testIndex = g_testEntries.countUntil!(a => a.name.toLower == entryTestName.toLower);
    if (testIndex == -1)
        testIndex = 0;

    testSelection = testIndex;

    entry = &g_testEntries[testIndex];
    test  = entry.createFcn();

    // Control the frame rate. One draw per monitor refresh.
    glfwSwapInterval(1);

    double time1     = glfwGetTime();
    double frameTime = 0.0;

    glClearColor(0.3f, 0.3f, 0.3f, 1.0f);

    while (!glfwWindowShouldClose(mainWindow))
    {
        glfwGetWindowSize(mainWindow, &g_camera.m_width, &g_camera.m_height);
        glViewport(0, 0, g_camera.m_width, g_camera.m_height);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ubyte mousebutton = 0;
        int mscroll = ui.scroll;
        ui.scroll = 0;

        double xd, yd;
        glfwGetCursorPos(mainWindow, &xd, &yd);
        int mousex = cast(int)xd;
        int mousey = cast(int)yd;

        mousey = g_camera.m_height - mousey;
        int leftButton = glfwGetMouseButton(mainWindow, GLFW_MOUSE_BUTTON_LEFT);

        if (leftButton == GLFW_PRESS)
            mousebutton |= MouseButton.left;

        imguiBeginFrame(mousex, mousey, mousebutton, mscroll);

        sSimulate();
        sInterface();

        // Measure speed
        double time2 = glfwGetTime();
        double alpha = 0.9f;
        frameTime = alpha * frameTime + (1.0 - alpha) * (time2 - time1);
        time1     = time2;

        char buffer[32];
        snprintf(buffer.ptr, 32, "%.1f ms", 1000.0 * frameTime);
        addGfxCmdText(5, 5, TextAlign.left, buffer, WHITE);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_DEPTH_TEST);
        imguiRender(g_camera.m_width, g_camera.m_height);

        glfwSwapBuffers(mainWindow);

        glfwPollEvents();
    }

    g_debugDraw.Destroy();
    imguiDestroy();
}
