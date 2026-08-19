#include "ui/ScanPathApplet.hpp"

#include "scan/RasterInfinityPath.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace {

void glfw_error(int, const char* desc) {
    std::fprintf(stderr, "GLFW: %s\n", desc ? desc : "");
}

ImVec2 map_xy(float x, float y, ImVec2 origin, ImVec2 size) {
    const float u = (x - kScanXMin) / (kScanXMax - kScanXMin);
    const float v = 1.0f - (y - kScanYMin) / (kScanYMax - kScanYMin);
    return ImVec2(origin.x + u * size.x, origin.y + v * size.y);
}

ImU32 time_color(float t, float t_max) {
    const float u = (t_max > 1e-6f) ? std::clamp(t / t_max, 0.0f, 1.0f) : 0.0f;
    float r = 0, g = 0, b = 0;
    ImGui::ColorConvertHSVtoRGB(0.58f - 0.58f * u, 0.75f, 1.0f, r, g, b);
    return IM_COL32(static_cast<int>(r * 255), static_cast<int>(g * 255),
                    static_cast<int>(b * 255), 230);
}

struct State {
    ScanPathParams draft;
    ScanPathParams applied;
    ScanPathResult path;
    bool have_path = false;
    bool show_centre = true;
    bool show_beam = true;
    std::string status = "Set parameters, then Go.";
    std::string error;

    bool apply_draft() {
        ScanPathParams next = draft;
        std::string err;
        if (!validate_scan_path_params(next, err)) {
            error = err;
            status = err;
            return false;
        }
        draft = next;
        applied = next;
        error.clear();
        status = "Parameters set.";
        return true;
    }

    void compute() {
        if (!apply_draft())
            return;
        path = generate_raster_infinity(applied);
        have_path = true;
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "Go: %d rows, %d centre updates, %d beam samples, %.2f s simulated",
                      path.rows, path.dwells, static_cast<int>(path.beam.size()),
                      path.duration);
        status = buf;
        if (!path.warning.empty()) {
            status += " — ";
            status += path.warning;
        }
    }

    void reset_plot() {
        path = {};
        have_path = false;
        status = "Plot cleared.";
        error.clear();
    }

    void draw_controls() {
        ImGui::BeginChild("controls", ImVec2(360, 0), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Raster + infinity scan");
        ImGui::Separator();
        ImGui::TextWrapped(
            "Coarse Lissajous (freqX=%.1f Hz, freqY=%.1f Hz) is sampled every "
            "time step (%.0f Hz). The scan centre walks a bidirectional X raster "
            "and is updated every DELTA Lissajous samples. Vertical gap starts "
            "at GAP0; ALPHA is added to that gap after each row. The raster "
            "runs until the beam reaches the top of the Y angular bounds.",
            kInfinityFreqX, kInfinityFreqY, kLissajousHz);

        ImGui::Spacing();
        ImGui::TextDisabled("Parameters");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("INC (rad)", &draft.inc, 0.005f, 0.02f, "%.4f");
        ImGui::SetItemTooltip("Basic increment applied to the scan centre (rad).");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("DELTA", &draft.delta, 1.0f, 5.0f, "%.0f");
        ImGui::SetItemTooltip(
            "Multiplier > 1. The Lissajous advances every time step; the raster "
            "centre moves every DELTA of those steps.");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("ALPHA (rad)", &draft.alpha, 0.005f, 0.02f, "%.4f");
        ImGui::SetItemTooltip(
            "Added to the vertical gap after each horizontal pass. "
            "ALPHA=0 keeps a uniform pitch of GAP0.");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("GAP0 (rad)", &draft.gap0, 0.005f, 0.02f, "%.4f");
        ImGui::SetItemTooltip("Initial vertical pitch between raster rows (rad).");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("AMPX (rad)", &draft.amp_x, 0.01f, 0.05f, "%.3f");
        ImGui::SetItemTooltip("Coarse X amplitude of the infinity (rad), 0..0.6.");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("AMPY (rad)", &draft.amp_y, 0.01f, 0.05f, "%.3f");
        ImGui::SetItemTooltip("Coarse Y amplitude of the infinity (rad), 0..0.6.");

        ImGui::Spacing();
        if (ImGui::Button("Set", ImVec2(100, 0)))
            apply_draft();
        ImGui::SameLine();
        if (ImGui::Button("Go", ImVec2(100, 0)))
            compute();
        ImGui::SameLine();
        if (ImGui::Button("Reset", ImVec2(100, 0)))
            reset_plot();

        ImGui::Spacing();
        ImGui::Checkbox("Show beam", &show_beam);
        ImGui::SameLine();
        ImGui::Checkbox("Show centre raster", &show_centre);

        ImGui::Spacing();
        ImGui::TextDisabled("Active");
        ImGui::Text("INC    %.4f rad", applied.inc);
        ImGui::Text("DELTA  %.0f   (centre every %.0f Lissajous ticks, %.3f s)",
                    applied.delta, applied.delta,
                    applied.delta / kLissajousHz);
        ImGui::Text("ALPHA  %.4f rad", applied.alpha);
        ImGui::Text("GAP0   %.4f rad", applied.gap0);
        ImGui::Text("AMP    (%.3f, %.3f) rad", applied.amp_x, applied.amp_y);
        ImGui::TextDisabled("Y gaps: GAP0, then + ALPHA each row");
        ImGui::TextDisabled("centre FOV  X[%.3f, %.3f]  Y[%.3f, %.3f]",
                            kScanXMin + applied.amp_x, kScanXMax - applied.amp_x,
                            kScanYMin + applied.amp_y, kScanYMax - applied.amp_y);

        if (have_path) {
            ImGui::Spacing();
            ImGui::TextDisabled("Result");
            ImGui::Text("rows %d   centre updates %d", path.rows, path.dwells);
            ImGui::Text("beam samples %d", static_cast<int>(path.beam.size()));
            ImGui::Text("duration %.3f s", path.duration);
            if (!path.row_gaps.empty()) {
                ImGui::TextDisabled("Y gaps (rad)");
                const int n = static_cast<int>(path.row_gaps.size());
                const int show = std::min(n, 8);
                std::string gaps;
                for (int i = 0; i < show; ++i) {
                    char piece[32];
                    std::snprintf(piece, sizeof(piece), "%s%.4f",
                                  i ? ", " : "", path.row_gaps[static_cast<std::size_t>(i)]);
                    gaps += piece;
                }
                if (n > show)
                    gaps += ", ...";
                ImGui::TextWrapped("%s", gaps.c_str());
            }
        }

        ImGui::Spacing();
        if (!error.empty())
            ImGui::TextColored(ImVec4(1, 0.4f, 0.35f, 1), "%s", error.c_str());
        else
            ImGui::TextWrapped("%s", status.c_str());

        ImGui::EndChild();
    }

    void draw_plot() {
        ImGui::BeginChild("plot", ImVec2(0, 0), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Path  (mechanical XY, rad)");
        ImGui::SameLine();
        ImGui::TextDisabled("cyan = start   red = end");
        ImGui::Separator();

        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const float pad = 8.0f;
        const float legend_h = 28.0f;
        const float side = std::max(120.0f, std::min(avail.x - pad, avail.y - pad - legend_h));
        const ImVec2 size(side, side);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ImVec2 p1(p0.x + size.x, p0.y + size.y);

        dl->AddRectFilled(p0, p1, IM_COL32(18, 20, 26, 255));
        dl->AddRect(p0, p1, IM_COL32(70, 80, 95, 255));

        const ImVec2 neu = map_xy(kScanNeutralX, kScanNeutralY, p0, size);
        dl->AddLine(ImVec2(neu.x, p0.y), ImVec2(neu.x, p1.y), IM_COL32(70, 90, 70, 160));
        dl->AddLine(ImVec2(p0.x, neu.y), ImVec2(p1.x, neu.y), IM_COL32(70, 90, 70, 160));

        if (have_path) {
            const ImVec2 c0 = map_xy(path.cx_min, path.cy_max, p0, size);
            const ImVec2 c1 = map_xy(path.cx_max, path.cy_min, p0, size);
            dl->AddRect(c0, c1, IM_COL32(90, 110, 80, 180), 0.0f, 0, 1.0f);

            if (show_beam && path.beam.size() >= 2) {
                const float t_max = path.beam.back().t;
                const int n = static_cast<int>(path.beam.size());
                const int stride = std::max(1, n / 25000);
                ImVec2 prev = map_xy(path.beam[0].x, path.beam[0].y, p0, size);
                for (int i = stride; i < n; i += stride) {
                    const ImVec2 cur = map_xy(path.beam[static_cast<std::size_t>(i)].x,
                                              path.beam[static_cast<std::size_t>(i)].y,
                                              p0, size);
                    dl->AddLine(prev, cur, time_color(path.beam[static_cast<std::size_t>(i)].t, t_max), 1.4f);
                    prev = cur;
                }
            }

            if (show_centre && path.centre.size() >= 2) {
                ImVec2 prev = map_xy(path.centre[0].x, path.centre[0].y, p0, size);
                for (std::size_t i = 1; i < path.centre.size(); ++i) {
                    const ImVec2 cur = map_xy(path.centre[i].x, path.centre[i].y, p0, size);
                    dl->AddLine(prev, cur, IM_COL32(240, 240, 245, 200), 1.8f);
                    prev = cur;
                }
                dl->AddCircleFilled(
                    map_xy(path.centre.front().x, path.centre.front().y, p0, size),
                    4.0f, IM_COL32(80, 220, 255, 255));
                dl->AddCircleFilled(
                    map_xy(path.centre.back().x, path.centre.back().y, p0, size),
                    4.0f, IM_COL32(255, 90, 80, 255));
            }
        }

        ImGui::Dummy(size);
        ImGui::TextDisabled("X [%.2f, %.2f]   Y [%.2f, %.2f]   + = neutral",
                            kScanXMin, kScanXMax, kScanYMin, kScanYMax);
        ImGui::EndChild();
    }

    void draw() {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::Begin("Scan Path Simulator", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoSavedSettings);
        draw_controls();
        ImGui::SameLine();
        draw_plot();
        ImGui::End();
    }
};

}  // namespace

int ScanPathApplet::run(std::atomic<bool>* keep_running) {
    glfwSetErrorCallback(glfw_error);
    if (!glfwInit()) {
        std::fprintf(stderr, "glfwInit failed\n");
        return 1;
    }

#if defined(__APPLE__)
    const char* glsl = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#else
    const char* glsl = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    GLFWwindow* window = glfwCreateWindow(1280, 800, "Scan Path Simulator", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 0.0f;
    style.FrameRounding = 3.0f;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl);

    State state;

    while (!glfwWindowShouldClose(window)) {
        if (keep_running && !keep_running->load())
            glfwSetWindowShouldClose(window, GLFW_TRUE);

        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        state.draw();
        ImGui::Render();

        int w = 0, h = 0;
        glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
