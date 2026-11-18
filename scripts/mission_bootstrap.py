"""Select a ROS overlay and attach the Mission Control bridge on the Jetson."""

if "detect_toss_workflow" not in globals():  # SSH sends these modules together.
    from mission_runtime import PRIVATE, native_overlay_changed, prepare_session, private_base, select_workspace
    from mission_toss import TossBench, attach_toss, detect_toss_workflow, toss_bench_installed


def bridge_selection(profile, domain, transition, preferred):
    if profile == "toss" or (profile in ("auto", "monitor") and transition != "body"
                             and detect_toss_workflow(domain)):
        return "toss", PRIVATE
    return "control", select_workspace(preferred, monitor_only=profile == "monitor")


def bridge_context(profile, domain, transition, workflow, *, allow_start=True):
    monitor = profile == "monitor"
    context = dict(follow_workflow=profile in ("auto", "monitor"), toss_monitor=monitor, toss_domain=domain)
    if workflow == "toss":
        attach_toss(domain)
        context.update(toss_bench=TossBench(), read_only="모니터링 전용" if monitor else "Self-tossing",
                       private_arm=True)
        return context

    # A shared bench remains available to both operators until someone chooses a task.
    start = (allow_start and not monitor and profile != "toss" and domain is None and
             (transition == "body" or (transition == "first" and not toss_bench_installed())))
    status, reason = prepare_session(monitor, domain, start_if_idle=start)
    if transition != "passive" and native_overlay_changed(reason):
        raise SystemExit(75)
    context.update(startup_status=status, read_only=reason, private_arm=private_base() is not None)
    return context
