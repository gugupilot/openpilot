#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
#include <poll.h>
#include <sys/mman.h>
#include "json11.hpp"
#include <fstream>
#include "common/util.h"
#include "common/swaglog.h"
#include "common/visionimg.h"
#include "common/utilpp.h"
#include "ui.hpp"
#include "paint.hpp"

std::map<std::string, int> LS_TO_IDX = {{"off", 0}, {"audible", 1}, {"silent", 2}};
std::map<std::string, int> DF_TO_IDX = {{"traffic", 0}, {"relaxed", 1}, {"roadtrip", 2}, {"auto", 3}};

extern volatile sig_atomic_t do_exit;

int write_param_float(float param, const char* param_name, bool persistent_param) {
  char s[16];
  int size = snprintf(s, sizeof(s), "%f", param);
  return write_db_value(param_name, s, size < sizeof(s) ? size : sizeof(s), persistent_param);
}

void ui_init(UIState *s) {
  s->sm = new SubMaster({"model", "controlsState", "uiLayoutState", "liveCalibration", "radarState", "thermal",
                         "health", "carParams", "ubloxGnss", "driverState", "dMonitoringState", "carState", "liveMpc", "liveParameters"});

  s->started = false;
  s->status = STATUS_OFFROAD;
  s->scene.satelliteCount = -1;
  read_param(&s->is_metric, "IsMetric");

  s->fb = framebuffer_init("ui", 0, true, &s->fb_w, &s->fb_h);
  assert(s->fb);

  ui_nvg_init(s);
}

static void ui_init_vision(UIState *s) {
  // Invisible until we receive a calibration message.
  s->scene.world_objects_visible = false;

  for (int i = 0; i < UI_BUF_COUNT; i++) {
    if (s->khr[i] != 0) {
      visionimg_destroy_gl(s->khr[i], s->priv_hnds[i]);
      glDeleteTextures(1, &s->frame_texs[i]);
    }

    VisionImg img = {
      .fd = s->stream.bufs[i].fd,
      .format = VISIONIMG_FORMAT_RGB24,
      .width = s->stream.bufs_info.width,
      .height = s->stream.bufs_info.height,
      .stride = s->stream.bufs_info.stride,
      .bpp = 3,
      .size = s->stream.bufs_info.buf_len,
    };
#ifndef QCOM
    s->priv_hnds[i] = s->stream.bufs[i].addr;
#endif
    s->frame_texs[i] = visionimg_to_gl(&img, &s->khr[i], &s->priv_hnds[i]);

    glBindTexture(GL_TEXTURE_2D, s->frame_texs[i]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // BGR
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, GL_GREEN);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, GL_RED);
  }
  assert(glGetError() == GL_NO_ERROR);
}

static void send_ls(UIState *s, int status) {
  capnp::MallocMessageBuilder msg;
  auto event = msg.initRoot<cereal::Event>();
  event.setLogMonoTime(nanos_since_boot());
  auto lsStatus = event.initLaneSpeedButton();
  lsStatus.setStatus(status);
  s->pm->send("laneSpeedButton", msg);
}

static void send_df(UIState *s, int status) {
  capnp::MallocMessageBuilder msg;
  auto event = msg.initRoot<cereal::Event>();
  event.setLogMonoTime(nanos_since_boot());
  auto dfStatus = event.initDynamicFollowButton();
  dfStatus.setStatus(status);
  s->pm->send("dynamicFollowButton", msg);
}

static void send_ml(UIState *s, bool enabled) {
  capnp::MallocMessageBuilder msg;
  auto event = msg.initRoot<cereal::Event>();
  event.setLogMonoTime(nanos_since_boot());
  auto mlStatus = event.initModelLongButton();
  mlStatus.setEnabled(enabled);
  s->pm->send("modelLongButton", msg);
}

static bool handle_ls_touch(UIState *s, int touch_x, int touch_y) {
  //lsButton manager
  int padding = 40;
  int btn_x_1 = 1660 - 200;
  int btn_x_2 = 1660 - 50;
  if ((btn_x_1 - padding <= touch_x) && (touch_x <= btn_x_2 + padding) && (855 - padding <= touch_y)) {
    printf("ls button touched!\n");
    s->scene.lsButtonStatus++;
    if (s->scene.lsButtonStatus > 2) { s->scene.lsButtonStatus = 0; }
    send_ls(s, s->scene.lsButtonStatus);
    return true;
  }
  return false;
}

static bool handle_df_touch(UIState *s, int touch_x, int touch_y) {
  //dfButton manager
  int padding = 40;
  if ((1660 - padding <= touch_x) && (855 - padding <= touch_y)) {
    printf("df button touched!\n");
    s->scene.dfButtonStatus++;
    if (s->scene.dfButtonStatus > 3) { s->scene.dfButtonStatus = 0; }
    send_df(s, s->scene.dfButtonStatus);
    return true;
   }
  return false;
 }

static bool handle_ml_touch(UIState *s, int touch_x, int touch_y) {
  //mlButton manager
  int padding = 40;
  int btn_w = 500;
  int btn_h = 138;
  int xs[2] = {1920 / 2 - btn_w / 2, 1920 / 2 + btn_w / 2};
  int y_top = 915 - btn_h / 2;
  if (xs[0] <= touch_x + padding && touch_x - padding <= xs[1] && y_top - padding <= touch_y) {
    printf("ml button touched!\n");
    s->scene.mlButtonEnabled = !s->scene.mlButtonEnabled;
    send_ml(s, s->scene.mlButtonEnabled);
    return true;
  }
  return false;
}

static bool handle_SA_touched(UIState *s, int touch_x, int touch_y) {
  if (s->active_app == cereal::UiLayoutState::App::NONE) {  // if onroad (not settings or home)
    if ((s->awake && s->vision_connected && s->status != STATUS_STOPPED) || s->ui_debug) {  // if car started or debug mode
      if (handle_df_touch(s, touch_x, touch_y)) { return true; }  // only allow one button to be pressed at a time
      if (handle_ls_touch(s, touch_x, touch_y)) { return true; }
      if (handle_ml_touch(s, touch_x, touch_y)) { return true; }
    }
  }
  return false;
}

void ui_update_vision(UIState *s) {

  if (!s->vision_connected && s->started) {
    const VisionStreamType type = s->scene.frontview ? VISION_STREAM_RGB_FRONT : VISION_STREAM_RGB_BACK;
    int err = visionstream_init(&s->stream, type, true, nullptr);
    if (err == 0) {
      ui_init_vision(s);
      s->vision_connected = true;
    }
  }

  if (s->vision_connected) {
    if (!s->started) goto destroy;

    // poll for a new frame
    struct pollfd fds[1] = {{
      .fd = s->stream.ipc_fd,
      .events = POLLOUT,
    }};
    int ret = poll(fds, 1, 100);
    if (ret > 0) {
      if (!visionstream_get(&s->stream, nullptr)) goto destroy;
    }
  }

  return;

destroy:
  visionstream_destroy(&s->stream);
  s->vision_connected = false;
}

static inline void fill_path_points(const cereal::ModelData::PathData::Reader &path, float *points) {
  const capnp::List<float>::Reader &poly = path.getPoly();
  for (int i = 0; i < path.getValidLen(); i++) {
    points[i] = poly[0] * (i * i * i) + poly[1] * (i * i) + poly[2] * i + poly[3];
  }
}

void update_sockets(UIState *s) {

  UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  // poll sockets
  if (sm.update(0) == 0){
    return;
  }

  if (s->started && sm.updated("controlsState")) {
    auto event = sm["controlsState"];
    scene.controls_state = event.getControlsState();

    s->scene.angleSteers = scene.controls_state.getAngleSteers();
    s->scene.steerOverride= scene.controls_state.getSteerOverride();
    s->scene.output_scale = scene.controls_state.getLateralControlState().getPidState().getOutput();
    s->scene.angleSteersDes = scene.controls_state.getAngleSteersDes();

    auto alert_sound = scene.controls_state.getAlertSound();
    if (scene.alert_type.compare(scene.controls_state.getAlertType()) != 0) {
      if (alert_sound == AudibleAlert::NONE) {
        s->sound->stop();
      } else {
        s->sound->play(alert_sound);
      }
    }
    scene.alert_text1 = scene.controls_state.getAlertText1();
    scene.alert_text2 = scene.controls_state.getAlertText2();
    scene.alert_size = scene.controls_state.getAlertSize();
    scene.alert_type = scene.controls_state.getAlertType();
    auto alertStatus = scene.controls_state.getAlertStatus();
    if (alertStatus == cereal::ControlsState::AlertStatus::USER_PROMPT) {
      s->status = STATUS_WARNING;
    } else if (alertStatus == cereal::ControlsState::AlertStatus::CRITICAL) {
      s->status = STATUS_ALERT;
    } else if (scene.controls_state.getEnabled()){
      s->status = (s->longitudinal_control)? STATUS_ENGAGED_OPLONG:STATUS_ENGAGED;
    }
    else {
      s->status = STATUS_DISENGAGED;
    }

    float alert_blinkingrate = scene.controls_state.getAlertBlinkingRate();
    if (alert_blinkingrate > 0.) {
      if (s->alert_blinked) {
        if (s->alert_blinking_alpha > 0.0 && s->alert_blinking_alpha < 1.0) {
          s->alert_blinking_alpha += (0.05*alert_blinkingrate);
        } else {
          s->alert_blinked = false;
        }
      } else {
        if (s->alert_blinking_alpha > 0.25) {
          s->alert_blinking_alpha -= (0.05*alert_blinkingrate);
        } else {
          s->alert_blinking_alpha += 0.25;
          s->alert_blinked = true;
        }
      }
    }
  }

  if (sm.updated("liveParameters")) {
    //scene.liveParams = sm["liveParameters"].getLiveParameters();
    auto data = sm["liveParameters"].getLiveParameters();    
    s->scene.steerRatio=data.getSteerRatio();
  }
  
  if (sm.updated("radarState")) {
    auto data = sm["radarState"].getRadarState();
    scene.lead_data[0] = data.getLeadOne();
    scene.lead_data[1] = data.getLeadTwo();
    s->scene.lead_v_rel = scene.lead_data[0].getVRel();
    s->scene.lead_d_rel = scene.lead_data[0].getDRel();
    s->scene.lead_status = scene.lead_data[0].getStatus();
  }
  if (sm.updated("liveCalibration")) {
    scene.world_objects_visible = true;
    auto extrinsicl = sm["liveCalibration"].getLiveCalibration().getExtrinsicMatrix();
    for (int i = 0; i < 3 * 4; i++) {
      scene.extrinsic_matrix.v[i] = extrinsicl[i];
    }
  }
  if (sm.updated("model")) {
    scene.model = sm["model"].getModel();
    fill_path_points(scene.model.getPath(), scene.path_points);
    fill_path_points(scene.model.getLeftLane(), scene.left_lane_points);
    fill_path_points(scene.model.getRightLane(), scene.right_lane_points);
  }
  if (sm.updated("uiLayoutState")) {
    auto data = sm["uiLayoutState"].getUiLayoutState();
    s->active_app = data.getActiveApp();
    scene.uilayout_sidebarcollapsed = data.getSidebarCollapsed();
  }
  if (sm.updated("thermal")) {
    scene.thermal = sm["thermal"].getThermal();
  }
  if (sm.updated("ubloxGnss")) {
    auto data = sm["ubloxGnss"].getUbloxGnss();
    if (data.which() == cereal::UbloxGnss::MEASUREMENT_REPORT) {
      scene.satelliteCount = data.getMeasurementReport().getNumMeas();
    }
  }
  if (sm.updated("health")) {
    auto health = sm["health"].getHealth();
    scene.hwType = health.getHwType();
    s->ignition = health.getIgnitionLine() || health.getIgnitionCan();
  } else if ((s->sm->frame - s->sm->rcv_frame("health")) > 5*UI_FREQ) {
    scene.hwType = cereal::HealthData::HwType::UNKNOWN;
  }
  if (sm.updated("carParams")) {
    s->longitudinal_control = sm["carParams"].getCarParams().getOpenpilotLongitudinalControl();
  }
  if (sm.updated("driverState")) {
    scene.driver_state = sm["driverState"].getDriverState();
  }
  if (sm.updated("dMonitoringState")) {
    scene.dmonitoring_state = sm["dMonitoringState"].getDMonitoringState();
    scene.is_rhd = scene.dmonitoring_state.getIsRHD();
    scene.frontview = scene.dmonitoring_state.getIsPreview();
  } else if ((sm.frame - sm.rcv_frame("dMonitoringState")) > UI_FREQ/2) {
    scene.frontview = false;
  }

  if (sm.updated("carState")) {
    auto data = sm["carState"].getCarState();
    if(scene.leftBlinker!=data.getLeftBlinker() || scene.rightBlinker!=data.getRightBlinker()){
      scene.blinker_blinkingrate = 50;
    }
    scene.brakeLights = data.getBrakeLights();
    scene.leftBlinker = data.getLeftBlinker();
    scene.rightBlinker = data.getRightBlinker();
    scene.leftblindspot = data.getLeftBlindspot();
    scene.rightblindspot = data.getRightBlindspot();
  } 

  s->started = scene.thermal.getStarted() || scene.frontview;
}

void ui_update(UIState *s) {

  update_sockets(s);
  ui_update_vision(s);

  // Handle onroad/offroad transition
  if (!s->started && s->status != STATUS_OFFROAD) {
    s->status = STATUS_OFFROAD;
    s->active_app = cereal::UiLayoutState::App::HOME;
    s->scene.uilayout_sidebarcollapsed = false;
  } else if (s->started && s->status == STATUS_OFFROAD) {
    s->status = STATUS_DISENGAGED;
    s->started_frame = s->sm->frame;

    s->active_app = cereal::UiLayoutState::App::NONE;
    s->scene.uilayout_sidebarcollapsed = true;
    s->alert_blinked = false;
    s->alert_blinking_alpha = 1.0;
    s->scene.alert_size = cereal::ControlsState::AlertSize::NONE;
  }

  // Handle controls timeout
  if (s->started && !s->scene.frontview && ((s->sm)->frame - s->started_frame) > 5*UI_FREQ) {
    if ((s->sm)->rcv_frame("controlsState") < s->started_frame) {
      // car is started, but controlsState hasn't been seen at all
      s->scene.alert_text1 = "openpilot Unavailable";
      s->scene.alert_text2 = "Waiting for controls to start";
      s->scene.alert_size = cereal::ControlsState::AlertSize::MID;
    } else if (((s->sm)->frame - (s->sm)->rcv_frame("controlsState")) > 5*UI_FREQ) {
      // car is started, but controls is lagging or died
      if (s->scene.alert_text2 != "Controls Unresponsive") {
        s->sound->play(AudibleAlert::CHIME_WARNING_REPEAT);
        LOGE("Controls unresponsive");
      }

      s->scene.alert_text1 = "TAKE CONTROL IMMEDIATELY";
      s->scene.alert_text2 = "Controls Unresponsive";
      s->scene.alert_size = cereal::ControlsState::AlertSize::FULL;
      s->status = STATUS_ALERT;
    }
  }

  // Read params
  if ((s->sm)->frame % (5*UI_FREQ) == 0) {
    read_param(&s->is_metric, "IsMetric");
  } else if ((s->sm)->frame % (6*UI_FREQ) == 0) {
    int param_read = read_param(&s->last_athena_ping, "LastAthenaPingTime");
    if (param_read != 0) { // Failed to read param
      s->scene.athenaStatus = NET_DISCONNECTED;
    } else if (nanos_since_boot() - s->last_athena_ping < 70e9) {
      s->scene.athenaStatus = NET_CONNECTED;
    } else {
      s->scene.athenaStatus = NET_ERROR;
    }
  }
}
