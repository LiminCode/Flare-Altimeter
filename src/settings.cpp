#include "settings.h"

#include <Preferences.h>

#include "flare_config.h"

namespace settings {

Settings s;

namespace {
Preferences prefs;
}

void save() {
  prefs.begin("flare", false);
  prefs.putBytes("qref", &s.mountingRef, sizeof(s.mountingRef));
  prefs.putBool("refset", s.refStored);
  prefs.putBool("corr", s.tiltCorrection);
  prefs.putFloat("conth", s.continuousAtCm);
  prefs.putFloat("bstep", s.beepStepCm);
  prefs.end();
}

void load() {
  prefs.begin("flare", true);   // read-only

  flare::Quat stored{1.0f, 0.0f, 0.0f, 0.0f};
  const bool haveBlob =
      prefs.getBytesLength("qref") == sizeof(stored) &&
      prefs.getBytes("qref", &stored, sizeof(stored)) == sizeof(stored);
  s.refStored = haveBlob && prefs.getBool("refset", false);
  if (s.refStored) s.mountingRef = stored;

  s.tiltCorrection = prefs.getBool("corr", true);

  // Everything read back gets validated rather than trusted. A stored value
  // outlives the code that wrote it: a firmware update, a half-written record, or
  // a stale key from an older build all deliver a number the running code never
  // chose, and a bad step size silently divides the whole beep ladder by
  // something absurd without raising anything.
  // 中文：读回来的值一律校验，不直接信任。存进 NVS 的值会活得比写它的代码更久 ——
  //       固件升级、写到一半掉电、旧版本残留的键，都会给出一个当前代码从没选过的数。
  //       尤其步长：一个脏值会让整条响声阶梯按荒谬的间隔切分，而且不报任何错。
  s.continuousAtCm = flare::clampContinuousAtCm(
      prefs.getFloat("conth", s.continuousAtCm));
  s.beepStepCm = flare::snapBeepStepCm(prefs.getFloat("bstep", s.beepStepCm));

  prefs.end();
}

}  // namespace settings
