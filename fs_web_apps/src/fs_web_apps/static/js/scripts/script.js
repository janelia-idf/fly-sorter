$(function() {
    // Initialize variables
    var vialsToRun = new Object();
    var outputMoving = false;
    var timer;
    var millisecondsInSecond = 1000;
    var running = false;
    var secondsPerVial = 353;
    var secondsOverhead = 83;
    var timeStart = new Date;
    var timeEnd = new Date;
    var timerInitializeWatchdog;
    var watchdogCount;
    var watchdogCountLimit = 2;
    var vialTrainingGenders = new Object();

    // jQuery UI
    // tabs
    var tabs = $("#tabs").tabs({heightStyle: "fill"});
    $(window).resize(function() {
        tabs.tabs('refresh');
    });
    $(':button').button();

    // Make checkbox lose focus after unclicking
    $('[type=checkbox]').change(function() {
        if(!$(this).is(':checked')) {
            $(this).blur();
        }
    });

    // ROS Setup
    var ros = new ROSLIB.Ros({
        // url : 'ws://localhost:9090'
        url : 'ws://' + location.hostname + ':9090'
    });

    // ROS Parameters
    var paramFlySorter = new ROSLIB.Param({
        ros : ros,
        name : '/fs_parameters'
    });
    var paramRunMode = new ROSLIB.Param({
        ros : ros,
        name : '/fs_smach/run_mode'
    });

    // ROS Topics
    var topicInitialize = new ROSLIB.Topic({
        ros : ros,
        name : '/fs_controls/initialize',
        messageType : 'std_msgs/Empty'
    });
    var topicHibernate = new ROSLIB.Topic({
        ros : ros,
        name : '/fs_controls/hibernate',
        messageType : 'std_msgs/Empty'
    });
    var topicInitializing = new ROSLIB.Topic({
        ros : ros,
        name : '/fs_status/initializing',
        messageType : 'std_msgs/Empty'
    });
    var topicInitialized = new ROSLIB.Topic({
        ros : ros,
        name : '/fs_status/initialized',
        messageType : 'std_msgs/Empty'
    });
    var topicRunParameters = new ROSLIB.Topic({
        ros : ros,
        name : '/fs_controls/set_run_parameters',
        messageType : 'fs_smach/RunParameters'
    });
    var topicHibernated = new ROSLIB.Topic({
        ros : ros,
        name : '/fs_status/hibernated',
        messageType : 'std_msgs/Empty'
    });
    var topicVialRunning = new ROSLIB.Topic({
        ros : ros,
        name : '/fs_status/vial/running',
        messageType : 'fs_smach/Vial'
    });
    var topicVialFinished = new ROSLIB.Topic({
        ros : ros,
        name : '/fs_status/vial/finished',
        messageType : 'fs_smach/Vial'
    });
    var topicRunFinished = new ROSLIB.Topic({
        ros : ros,
        name : '/fs_status/run_finished',
        messageType : 'fs_smach/RunSummary'
    });
    var topicRunAgain = new ROSLIB.Topic({
        ros : ros,
        name : '/fs_controls/run_again',
        messageType : 'std_msgs/Empty'
    });

    var messageEmpty = new ROSLIB.Message({
    });

    topicInitializing.subscribe(function(message) {
        $('.initializing').show();
        clearInterval(timerInitializeWatchdog);
    });
    topicInitialized.subscribe(function(message) {
        $('#btnOutputHome').button('disable');
        $('#btnOutputLoad').button('enable');
        $('input:checkbox[name=vialSelect]').prop('checked',false);
        $('input:checkbox[name=vialSelect]').button('refresh');
        $('input:checkbox[name=vialSelect]').each(function() {
            vialsToRun[$(this).prop('value')] = $(this).prop('checked');
        });
        $('.img_vial').attr("src","static/icons/x.png");
        $('.button.control').button('enable');
        $('#vialsToRun').buttonset('enable');
        $('#btnRunVials').button('disable');
        $('#btnHibernate').button('enable');
        outputMoving = false;
        running = false;
        tabs.tabs('disable',1);
        tabs.tabs('enable',2);
        tabs.tabs("option","active",2);
        tabs.tabs('disable',3);
        timer = setInterval(timerFunc,millisecondsInSecond);
        $('input:checkbox[name=male]').prop('checked',false);
        $('input:checkbox[name=male]').button('refresh');
        $('input:checkbox[name=female]').prop('checked',false);
        $('input:checkbox[name=female]').button('refresh');
        $('input:checkbox[name=mixed]').prop('checked',false);
        $('input:checkbox[name=mixed]').button('refresh');
        paramRunMode.get(function(value) {
            if (value.toLowerCase() == 'training') {
                $('#btnRunModeSorting').button('enable');
                $('#btnRunModeTraining').button('disable');
                $('.training').button('enable');
                $('.training').button().show();
                $('.genderSelect').buttonset('enable');
                $('.genderSelect').show();
                $('.genderSelect').buttonset('refresh');
                $('#genderDisplayRow').show();
                $('.vialGenderDisplay').css('visibility','hidden');
            } else {
                $('#btnRunModeSorting').button('disable');
                $('#btnRunModeTraining').button('enable');
                $('.training').button('disable');
                $('.training').button().hide();
                $('.genderSelect').buttonset('disable');
                $('.genderSelect').hide();
                $('.genderSelect').buttonset('refresh');
                $('#genderDisplayRow').hide();
                $('.vialGenderDisplay').css('visibility','hidden');
            };
            $('input:checkbox[name=vialSelect]').each(function() {
            var vial = $(this).prop('value');
                vialTrainingGenders[vial] = 'male';
                $('#' + vial + "_gender").attr("src","static/icons/male.png");
                $('#' + vial + "_male").button('disable').button('refresh');
                $('#' + vial + "_female").button('disable').button('refresh');
                $('#' + vial + "_mixed").button('disable').button('refresh');
                $('.genderSelect').buttonset('refresh');
            });
        });
    });
    topicHibernated.subscribe(function(message) {
        initializeGui();
    });
    topicVialRunning.subscribe(function(message) {
        $('#img_'+message.vial).attr("src","static/icons/circle_arrow.png");
    });
    topicVialFinished.subscribe(function(message) {
        $('#img_'+message.vial).attr("src","static/icons/check.png");
    });
    topicRunFinished.subscribe(function(message) {
        tabs.tabs('enable',3);
        tabs.tabs("option","active",3);
        tabs.tabs('disable',1);
        tabs.tabs('disable',2);

        var font_size = "1em";
        var run_data = JSON.parse(message.run_data);
        var tbl = prettyPrint(run_data, {
            // Config
            maxDepth: 5,
            styles: {
                'default': {
                    td: {
                        fontSize: font_size,
                        border: '4px solid #000'
                    },
                    th: {
                        fontSize: font_size,
                        border: '4px solid #000'
                    }
                }
            }
        });
        $("#runSummary").html(tbl);

        var set_data = JSON.parse(message.set_data);
        tbl = prettyPrint(set_data, {
            // Config
            maxDepth: 5,
            styles: {
                'default': {
                    td: {
                        fontSize: font_size,
                        border: '4px solid #000'
                    },
                    th: {
                        fontSize: font_size,
                        border: '4px solid #000'
                    }
                }
            }
        });
        $("#setSummary").html(tbl);

        var fly_sorter_data = JSON.parse(message.fly_sorter_data);
        tbl = prettyPrint(fly_sorter_data, {
            // Config
            maxDepth: 5,
            styles: {
                'default': {
                    td: {
                        fontSize: font_size,
                        border: '4px solid #000'
                    },
                    th: {
                        fontSize: font_size,
                        border: '4px solid #000'
                    }
                }
            }
        });
        $("#flySorterSummary").html(tbl);
    });

    // ROS Actions
    var actionGoToOutputPos = new ROSLIB.ActionClient({
        ros : ros,
        serverName : '/fs_actionlib/go_to_output_pos',
        actionName : '/fs_actionlib/GoToPosAction'
    });

    var goalGoToOutputPosHome = new ROSLIB.Goal({
        actionClient : actionGoToOutputPos,
        goalMessage : {
            motor_name : "output",
            position: "origin"
        }
    });

    var goalGoToOutputPosLoad = new ROSLIB.Goal({
        actionClient : actionGoToOutputPos,
        goalMessage : {
            motor_name : "output",
            position: "load"
            // position: "output_3"
        }
    });

    goalGoToOutputPosHome.on('result', function(result) {
        $('#btnOutputLoad').button('enable');
        if ($("input:checkbox[name=vialSelect]:checked").length > 0) {
            $('#btnRunVials').button('enable');
        };
        $('#btnHibernate').button('enable');
        outputMoving = false;
    });

    goalGoToOutputPosLoad.on('result', function(result) {
        $('#btnOutputHome').button('enable');
        if ($("input:checkbox[name=vialSelect]:checked").length > 0) {
            $('#btnRunVials').button('enable');
        };
        $('#btnHibernate').button('enable');
        outputMoving = false;
    });

    // GUI Button Controls

    // Parameters Tab
    $('#btnParameters').click(function() {
        $(this).blur();
        paramFlySorter.get(function(params) {
            var tbl = prettyPrint(params, {
                // Config
                maxDepth: 5,
                styles: {
                    'default': {
                        td: {
                            fontSize: '1em',
                            border: '4px solid #000'
                        },
                        th: {
                            fontSize: '1em',
                            border: '4px solid #000'
                        }
                    }
                }
            });
            $("#parameters").html(tbl);
        });
    });

    // Initialize Tab
    var timerInitializeWatchdogFunc = function() {
        console.log('watchdogCount = ' + watchdogCount);
        if(watchdogCount >= (watchdogCountLimit - 1)) {
            clearInterval(timerInitializeWatchdog);
            $('.initializeError').show();
        };
        watchdogCount = watchdogCount + 1;
    };
    $('#btnInitialize').click(function() {
        $(this).blur();
        $(this).button('disable');
        $(this).hide();
        $('.reminders').hide();
        watchdogCount = 0;
        timerInitializeWatchdog = setInterval(timerInitializeWatchdogFunc,millisecondsInSecond);
        topicInitialize.publish(messageEmpty);
    });
    $('#btnReload').click(function() {
        location.reload(true);
    });
    $('#btnHibernate').click(function() {
        topicHibernate.publish(messageEmpty);
        $('#btnInitialize').hide();
        $('.reminders').hide();
        $('.initializing').hide();
        $('.initializeError').hide();
        $('.hibernating').show();
        tabs.tabs('enable',1);
        tabs.tabs("option","active",1);
        tabs.tabs('disable',0);
        tabs.tabs('disable',2);
        tabs.tabs('disable',3);
    });
    $('#btnHibernate2').click(function() {
        topicHibernate.publish(messageEmpty);
        $('#btnInitialize').hide();
        $('.reminders').hide();
        $('.initializing').hide();
        $('.initializeError').hide();
        $('.hibernating').show();
        tabs.tabs('enable',1);
        tabs.tabs("option","active",1);
        tabs.tabs('disable',0);
        tabs.tabs('disable',2);
        tabs.tabs('disable',3);
    });
    $('#btnRunAgain').click(function() {
        topicRunAgain.publish(messageEmpty);
    });

    // Controls Tab
    $('#btnOutputHome').click(function() {
        $(this).blur();
        $(this).button('disable');
        $('#btnOutputLoad').button('disable');
        $('#btnRunVials').button('disable');
        $('#btnHibernate').button('disable');
        outputMoving = true;
        goalGoToOutputPosHome.send();
    });
    $('#btnOutputLoad').click(function() {
        $(this).blur();
        $(this).button('disable');
        $('#btnOutputHome').button('disable');
        $('#btnRunVials').button('disable');
        $('#btnHibernate').button('disable');
        outputMoving = true;
        goalGoToOutputPosLoad.send();
    });

    $("#vialsToRun").buttonset();
    $("input:checkbox[name=vialSelect]").click(function() {
        var vial = $(this).prop("value");
        vialsToRun[vial] = $(this).prop("checked");
        if (($("input:checkbox[name=vialSelect]:checked").length > 0) && !outputMoving) {
            $('#btnRunVials').button('enable');
        } else {
            $('#btnRunVials').button('disable');
        };
        if ($(this).prop("checked")) {
            $('#img_'+vial).attr("src","static/icons/star.png");
            $('#' + vial + "_gender").css('visibility','visible');
            $('#' + vial + "_male").button('enable');
            $('#' + vial + "_female").button('enable');
            $('#' + vial + "_mixed").button('enable');
        } else {
            $('#img_'+vial).attr("src","static/icons/x.png");
            $('#' + vial + "_gender").css('visibility','hidden');
            $('#' + vial + "_male").button('disable');
            $('#' + vial + "_female").button('disable');
            $('#' + vial + "_mixed").button('disable');
        };
        $('.genderSelect').buttonset('refresh');
    });

    $("#genderSelectMale").buttonset();
    $("input:checkbox[name=male]").click(function() {
        $(this).blur();
        $(this).prop('checked',false);
        $(this).button('refresh');
        var vial = $(this).prop("value");
        vialTrainingGenders[vial] = 'male';
        $('#' + vial + "_gender").attr("src","static/icons/male.png");
    });

    $("#genderSelectFemale").buttonset();
    $("input:checkbox[name=female]").click(function() {
        $(this).blur();
        $(this).prop('checked',false);
        $(this).button('refresh');
        var vial = $(this).prop("value");
        vialTrainingGenders[vial] = 'female';
        $('#' + vial + "_gender").attr("src","static/icons/female.png");
    });

    $("#genderSelectMixed").buttonset();
    $("input:checkbox[name=mixed]").click(function() {
        $(this).blur();
        $(this).prop('checked',false);
        $(this).button('refresh');
        var vial = $(this).prop("value");
        vialTrainingGenders[vial] = 'mixed';
        $('#' + vial + "_gender").attr("src","static/icons/mixed.png");
    });

    $('#btnSelectAll').click(function() {
        $(this).blur();
        $("input:checkbox[name=vialSelect]").prop('checked',true);
        $('input:checkbox[name=vialSelect]').button('refresh');
        $('input:checkbox[name=vialSelect]').each(function() {
            var vial = $(this).prop('value');
            vialsToRun[vial] = true;
            $('#' + vial + "_gender").css('visibility','visible');
            $('#' + vial + "_male").button('enable');
            $('#' + vial + "_female").button('enable');
            $('#' + vial + "_mixed").button('enable');
        });
        $('.genderSelect').buttonset('refresh');
        if (!outputMoving) {
            $('#btnRunVials').button('enable');
        };
        $('.img_vial').attr("src","static/icons/star.png");
    });
    $('#btnSelectNone').click(function() {
        $(this).blur();
        $("input:checkbox[name=vialSelect]").prop('checked',false);
        $('input:checkbox[name=vialSelect]').button('refresh');
        $('input:checkbox[name=vialSelect]').each(function() {
            var vial = $(this).prop('value');
            vialsToRun[vial] = false;
            $('#' + vial + "_gender").css('visibility','hidden');
            $('#' + vial + "_male").button('disable');
            $('#' + vial + "_female").button('disable');
            $('#' + vial + "_mixed").button('disable');
        });
        $('.genderSelect').buttonset('refresh');
        $('#btnRunVials').button('disable');
        $('#btnhibernate').button('disable');
        $('.img_vial').attr("src","static/icons/x.png");
    });

    $('#btnRunVials').click(function() {
        $(this).blur();
        $('.button.output').button('disable');
        $('.button.control').button('disable');
        $('#vialsToRun').buttonset('disable');
        $('.genderSelect').buttonset('disable');
        $('.training').button('disable');
        var runParameters = new ROSLIB.Message({
            vials_to_run : JSON.stringify(vialsToRun),
            vial_training_genders: JSON.stringify(vialTrainingGenders)
        });
        topicRunParameters.publish(runParameters);
        running = true;
    });
    $('#btnStop').click(function() {
    });
    $('#btnRunModeSorting').click(function() {
        $(this).blur();
        paramRunMode.set('sorting');
        $('#btnRunModeSorting').button('disable');
        $('#btnRunModeTraining').button('enable');
        $('.training').button('disable');
        $('.training').button().hide();
        $('.genderSelect').buttonset('disable');
        $('.genderSelect').hide();
        $('#genderDisplayRow').hide();
        $('.vialGenderDisplay').css('visibility','hidden');
    });
    $('#btnRunModeTraining').click(function() {
        $(this).blur();
        paramRunMode.set('training');
        $('#btnRunModeSorting').button('enable');
        $('#btnRunModeTraining').button('disable');
        $('.training').button('enable');
        $('.training').button().show();
        $('.genderSelect').buttonset('enable');
        $('.genderSelect').show();
        $('#genderDisplayRow').show();
        $('input:checkbox[name=vialSelect]').each(function() {
            var vial = $(this).prop('value');
            if ($(this).prop('checked')) {
                $('#' + vial + "_gender").css('visibility','visible');
            } else {
                $('#' + vial + "_male").button('disable');
                $('#' + vial + "_female").button('disable');
                $('#' + vial + "_mixed").button('disable');
                $('.genderSelect').buttonset('refresh');
            };
        });
    });
    $('#btnAllMale').click(function() {
        $(this).blur();
        $('input:checkbox[name=male]').each(function() {
            var vial = $(this).prop('value');
            vialTrainingGenders[vial] = 'male';
            $('#' + vial + "_gender").attr("src","static/icons/male.png");
        });
    });
    $('#btnAllFemale').click(function() {
        $(this).blur();
        $('input:checkbox[name=female]').each(function() {
            var vial = $(this).prop('value');
            vialTrainingGenders[vial] = 'female';
            $('#' + vial + "_gender").attr("src","static/icons/female.png");
        });
    });
    $('#btnAllMixed').click(function() {
        $(this).blur();
        $('input:checkbox[name=mixed]').each(function() {
            var vial = $(this).prop('value');
            vialTrainingGenders[vial] = 'mixed';
            $('#' + vial + "_gender").attr("src","static/icons/mixed.png");
        });
    });

    // Initialize Gui
    var initializeGui = function() {
        tabs.tabs('enable',1);
        tabs.tabs("option","active",1);
        tabs.tabs('disable',2);
        tabs.tabs('disable',3);
        $("#parameters").html('');
        $('#btnParameters').button('enable');
        $('#btnInitialize').button('enable');
        $('#btnInitialize').show();
        $('.reminders').show();
        $('.initializing').hide();
        $('.hibernating').hide();
        $('#btnReload').button('enable');
        $('.initializeError').hide();
        $('#btnStop').button('enable');
        clearInterval(timer);
        clearInterval(timerInitializeWatchdog);
        $('#btnRunModeSorting').button('disable');
        $('#btnRunModeTraining').button('enable');
        $('.training').button('disable');
        $('.training').button().hide();
        $('.genderSelect').buttonset('disable');
        $('.genderSelect').hide();
        $('#genderDisplayRow').hide();
        $('#btnHibernate2').button('enable');
        $('#btnRunAgain').button('enable');
    };
    initializeGui();

    // Timer
    var setTimeEnd = function() {
        var vialsCheckedCount = $("input:checkbox[name=vialSelect]:checked").length;
        if(vialsCheckedCount > 0) {
            timeEnd.setTime(timeStart.getTime() + vialsCheckedCount*secondsPerVial*millisecondsInSecond + secondsOverhead*millisecondsInSecond);
        } else {
            timeEnd.setTime(timeStart.getTime());
        };
    };
    var timerFunc = function() {
        var timeNow = new Date();
        $('#timeNow').text(timeNow.toLocaleString());
        if (!running) {
            timeStart.setTime(timeNow.getTime());
            setTimeEnd();
            $('#timeStart').text(timeStart.toLocaleString());
            $('#timeEnd').text(timeEnd.toLocaleString());
        };
    };
});
