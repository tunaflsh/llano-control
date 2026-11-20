"""Fan control from labeled captures. RPM are targets, not tachometer readings."""
import math
import time
from .rgb import HidFeature, checked_state
from .core import Preview, number


def rpm_to_percent(rpm):
    """Experimental calibration: 40/50/60% = 1300/1550/1800 RPM.

    1% at 300 RPM was observed in physical manual mode. Remaining targets
    extrapolate the measured middle range and require physical validation.
    0 is handled as a whole-device power-off by the controller.
    """
    number(rpm,0,2800)
    if rpm == 0: return 0
    return max(1, min(100, round((rpm-300)/25)))


def fan_report(state, percent, take_control=False, power=None):
    checked_state(state)
    if type(percent) is not int or not 0 <= percent <= 100:
        raise ValueError('Porcentaje fuera de rango / Percentage out of range')
    if type(take_control) is not bool or (power is not None and type(power) is not bool):
        raise ValueError('Estado de control inválido / Invalid control state')
    body=bytes([int(take_control),percent,state[2] if power is None else int(not power)])+state[3:7]
    return body+bytes([(255-sum(body))&255])


def apply_fan(settings, transport=HidFeature):
    percent=settings['percent']; take=settings.get('take_control',False)
    power=settings.get('power') if take else None
    # Validate before opening hardware, even for a paused/manual response.
    fan_report(bytes.fromhex('80 01 00 03 04 00 ff 78'),percent,take,power)
    with transport() as device:
        before=device.state()
        if not take and (before[0]&8 or before[2]):
            return {'paused':True,'reason':'manual' if before[0]&8 else 'off'}
        report=fan_report(before,percent,take,power)
        changed=take or before[1:3]!=report[1:3]
        if changed:
            device.send(report);time.sleep(.05)
            after=device.state()
        else: after=before
        if after[0]&8:
            if not take: return {'paused':True,'reason':'manual'}
            raise OSError('No se recuperó el control del PC / PC control was not confirmed')
        if after[1:3]!=report[1:3] or after[3:7]!=before[3:7]:
            raise OSError('Consigna no confirmada o RGB cambió / Target not confirmed or RGB changed')
        return {'paused':False,'percent':after[1],'power':not bool(after[2]),'changed':changed}


def power_report(state, enabled):
    checked_state(state)
    if type(enabled) is not bool:
        raise ValueError('Estado de encendido inválido / Invalid power state')
    body=bytes([0,state[1],int(not enabled)])+state[3:7]
    return body+bytes([(255-sum(body))&255])


def apply_power(enabled, transport=HidFeature, *, toggle=False):
    if type(toggle) is not bool or (toggle and enabled is not None) or (not toggle and type(enabled) is not bool):
        raise ValueError('Estado de encendido inválido / Invalid power state')
    with transport() as device:
        before=device.state()
        if toggle:enabled=bool(before[2])
        report=power_report(before,enabled)
        changed=before[2]!=report[2]
        if changed:
            device.send(report);time.sleep(.05);after=device.state()
        else:after=before
        if after[2]!=report[2]:
            raise OSError('Encendido no confirmado / Power state not confirmed')
        if after[:2]!=before[:2] or after[3:7]!=before[3:7]:
            raise OSError('Cambió otro ajuste durante la operación / Another setting changed during operation')
        return {'power':not bool(after[2]),'changed':changed}


def apply_curve_off(transport=HidFeature):
    with transport() as device:
        before=device.state()
        if before[0]&8: return {'paused':True,'reason':'manual'}
        if before[2]: return {'power':False,'changed':False,'curve_off':True}
        report=power_report(before,False)
        device.send(report);time.sleep(.05);after=device.state()
        if after[2]!=1:
            raise OSError('Encendido no confirmado / Power state not confirmed')
        if after[:2]!=before[:2] or after[3:7]!=before[3:7]:
            raise OSError('Cambió otro ajuste durante la operación / Another setting changed during operation')
        return {'power':False,'changed':True,'curve_off':True}


class FanController:
    """Applied profile snapshot; UI edits have no effect until Apply is pressed."""
    def __init__(self,profile):
        import copy
        self.profile=copy.deepcopy(profile);self.preview=Preview()
        self.next_due=0.;self.initial=True;self.active=True;self.curve_off=False
        self.wake_since=None

    def plan(self,temp,now):
        if not self.active or now<self.next_due:return None
        if self.profile['power']:
            if (self.profile['mode']!='Manual' or self.profile.get('manual_thermal',False)) and (temp is None or not math.isfinite(temp)):
                self.active=False
                raise ValueError('Sin temperatura seleccionada: control pausado; se conserva la última consigna. / No selected temperature: control paused; last target retained.')
            rpm=self.preview.calculate(self.profile,temp)
        else: rpm=self.profile['rpm']
        if rpm == 0 and self.profile['power']:
            self.wake_since=None
            job={'operation':'curve_off'}
        elif self.curve_off and self.wake_since is None:
            self.wake_since=now
            job={'operation':'curve_off'}
        elif self.curve_off and now-self.wake_since < float(self.profile.get('hold',10)):
            job={'operation':'curve_off'}
        else:
            self.wake_since=None
            job={'percent':rpm_to_percent(rpm),'take_control':self.initial or self.curve_off}
            if job['take_control']: job['power']=self.profile['power']
        self.next_due=now+5
        return job

    def completed(self,result):
        self.initial=False
        self.curve_off=bool(result.get('curve_off'))
        if result.get('paused') or not result.get('power',True) and not result.get('curve_off'):self.active=False


def main():
    import fcntl,json,os,sys
    from pathlib import Path
    try:
        runtime=Path(os.environ.get('XDG_RUNTIME_DIR',f'/run/user/{os.getuid()}'))
        # Shared with RGB: serialize read-modify-write of the full state block.
        with (runtime/'llano-control-rgb.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            settings=json.loads(sys.argv[1])
            if settings.get('operation')=='read_power':
                with HidFeature() as device:result={'power':not bool(device.state()[2])}
            elif settings.get('operation')=='curve_off':result=apply_curve_off()
            elif settings.get('operation')=='toggle_power':result=apply_power(None,toggle=True)
            elif settings.get('operation')=='power':result=apply_power(settings['enabled'])
            else:result=apply_fan(settings)
        print(json.dumps(result))
    except Exception as error:
        print(str(error),file=sys.stderr);sys.exit(1)

if __name__=='__main__':main()
