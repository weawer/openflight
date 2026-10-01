#include <stdio.h>
#include <string.h>

#include "l3_ball_fit.h"
#include "l3_launch.h"
#include "l3_text.h"

void l3_launch_from_delivery(const l3_delivery_t *fit, uint32_t impactTimestampUs,
                             l3_launch_t *out)
{
    float dtS = (float)(int32_t)(impactTimestampUs - fit->timestampUs) * 1.0e-6F;

    memset(out, 0, sizeof(*out));
    out->points = fit->points;
    out->velocity = fit->velocity;
    out->launchPosition.x = fit->position.x + fit->velocity.x * dtS;
    out->launchPosition.y = fit->position.y + fit->velocity.y * dtS;
    out->launchPosition.z = fit->position.z + fit->velocity.z * dtS;
    out->speedMps = fit->speedMps;
    out->radialSpeedMps = fit->radialSpeedMps;
    out->residualM = fit->residualM;
    out->confidence = fit->confidence;
    out->speedValid = fit->speedValid;
    if (fit->pathValid) {
        out->hlaRad = fit->pathRad;
        out->hlaValid = 1U;
    }
    if (fit->attackValid) {
        out->vlaRad = fit->attackRad;
        out->vlaValid = 1U;
    }
}

int32_t l3_launch_format(const l3_launch_t *launch, char *out, uint32_t cap)
{
    char speedText[16];
    char radialText[16];
    char hlaText[16];
    char vlaText[16];
    char residualText[16];
    char confidenceText[16];
    char valid[4];
    char rmsText[16];
    uint32_t v = 0U;

    l3_text_fixed2(launch->speedMps, speedText, sizeof(speedText));
    l3_text_fixed2(launch->radialSpeedMps, radialText, sizeof(radialText));
    l3_text_degrees2(launch->hlaRad, hlaText, sizeof(hlaText));
    l3_text_degrees2(launch->vlaRad, vlaText, sizeof(vlaText));
    l3_text_fixed2(launch->residualM * 1000.0F, residualText, sizeof(residualText));
    l3_text_fixed2(launch->confidence, confidenceText, sizeof(confidenceText));
    l3_text_degrees2(launch->angleRmsRad, rmsText, sizeof(rmsText));
    if (launch->speedValid) {
        valid[v++] = 's';
    }
    if (launch->hlaValid) {
        valid[v++] = 'h';
    }
    if (launch->vlaValid) {
        valid[v++] = 'v';
    }
    valid[v] = '\0';
    return snprintf(out, cap,
                    "launch points=%u speed=%s radial=%s hla=%s vla=%s residualmm=%s conf=%s "
                    "angles=%u rms=%s why=%s valid=%s",
                    (unsigned)launch->points, speedText, radialText, hlaText, vlaText,
                    residualText, confidenceText, (unsigned)launch->anglesAccepted, rmsText,
                    l3_ball_fit_why_name(launch->angleWhy), (v > 0U) ? valid : "none");
}
