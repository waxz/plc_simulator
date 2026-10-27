USING Siemens.Simatic.Communication.OUC;

USING Siemens.Simatic.Communication.OpenUser;
USING Siemens.Simatic.Communication.MQTT;
USING Siemens.Simatic.Communication.Util;
USING System.Strings;
USING System.Serialization;

USING Simatic.Ax.IO.Input;
USING Simatic.Ax.IO.Output;


CONFIGURATION myPLC
    TASK Main(Interval := T#10ms, Priority := 16);
    PROGRAM P2 WITH Main : CommunicationProgram;

    VAR_GLOBAL
        sDONE, sERROR : BOOL;
        sSTATUS : WORD;
        sSendDONE, sSendERROR : BOOL;
        sSendSTATUS : WORD;
        sSendExecute, sSendBusy : BOOL;
        sSendDoneCount, sSendErrorCount : UDINT;
        sLastSendSTATUS : WORD;
        sReceiveNewData, sReceiveBusy, sReceiveERROR : BOOL;
        sReceiveSTATUS, sLastReceiveSTATUS : WORD;
        sReceiveLength : INT;
        sReceiveCount, sReceiveErrorCount : UDINT;
        sReceiveBuffer : ARRAY [0..63] OF BYTE;
        sCycleCount : UDINT;
        sConnectAttemptCount, sConnectDoneCount, sConnectErrorCount : UDINT;
        sLastConnectSTATUS : WORD;
        sConnectExecute, sConnectBusy : BOOL;
        sConnected : BOOL;
                tcpClient : TcpClient;
        tcp_praser:TCP_StreamParser;

    END_VAR
END_CONFIGURATION

PROGRAM CommunicationProgram
    VAR_EXTERNAL
        sDONE, sERROR : BOOL;
        sSTATUS : WORD;
        sSendDONE, sSendERROR : BOOL;
        sSendSTATUS : WORD;
        sSendExecute, sSendBusy : BOOL;
        sSendDoneCount, sSendErrorCount : UDINT;
        sLastSendSTATUS : WORD;
        sReceiveNewData, sReceiveBusy, sReceiveERROR : BOOL;
        sReceiveSTATUS, sLastReceiveSTATUS : WORD;
        sReceiveLength : INT;
        sReceiveCount, sReceiveErrorCount : UDINT;
        sReceiveBuffer : ARRAY [0..63] OF BYTE;
        sCycleCount : UDINT;
        sConnectAttemptCount, sConnectDoneCount, sConnectErrorCount : UDINT;
        sLastConnectSTATUS : WORD;
        sConnectExecute, sConnectBusy : BOOL;
        sConnected : BOOL;

        MI_staion_1: Station_1_Input;
        MO_staion_1: Station_1_Output;
       MO_pubBuffer: ARRAY[0..UDINT#255] OF BYTE;
       MO_pubMirror: Station_1_Output;

       MO_pubLen: UDINT;

    END_VAR

    VAR_EXTERNAL
                tcpClient : TcpClient;
        tcp_praser:TCP_StreamParser;
        END_VAR
    VAR
        // tcpClient : TcpClient;
        // tcp_praser:TCP_StreamParser;
        brokerIp : Siemens.Simatic.Communication.IPConfiguration.IPv4 := [BYTE#192, BYTE#168, BYTE#10, BYTE#11];
        data : ARRAY [0..4] OF BYTE := [BYTE#72, BYTE#69, BYTE#76, BYTE#76, BYTE#79];
        payloadQueued : BOOL := FALSE;
    END_VAR
    VAR_TEMP
        i : DINT;
    END_VAR

    sCycleCount := sCycleCount + UDINT#1;

    tcpClient.Configure(
        remoteAddress := brokerIp,
        remotePort := UINT#5001,
        connectionID := WORD#2,
        hardwareID := UINT#64,
        period := T#10MS
    );

    // IF NOT payloadQueued THEN
    //     tcpClient.QueueSend(
    //         data := data,
    //         dataLength := UDINT#5
    //     );
    //     payloadQueued := TRUE;
    // END_IF;

    tcpClient.Run();

    sDONE := tcpClient.ConnectDone;
    sERROR := tcpClient.ConnectError;
    sSTATUS := tcpClient.ConnectStatus;
    sConnectExecute := tcpClient.ConnectExecute;
    sConnectBusy := tcpClient.ConnectBusy;
    sConnectAttemptCount := tcpClient.ConnectAttemptCount;
    sConnectDoneCount := tcpClient.ConnectDoneCount;
    sConnectErrorCount := tcpClient.ConnectErrorCount;
    sLastConnectSTATUS := tcpClient.LastConnectStatus;
    sConnected := tcpClient.IsConnected;

    sSendDONE := tcpClient.SendDone;
    sSendERROR := tcpClient.SendError;
    sSendSTATUS := tcpClient.SendStatus;
    sSendExecute := tcpClient.SendExecute;
    sSendBusy := tcpClient.SendBusy;
    sSendDoneCount := tcpClient.SendDoneCount;
    sSendErrorCount := tcpClient.SendErrorCount;
    sLastSendSTATUS := tcpClient.LastSendStatus;

    sReceiveNewData := tcpClient.ReceiveNewData;
    sReceiveBusy := tcpClient.ReceiveBusy;
    sReceiveERROR := tcpClient.ReceiveError;
    sReceiveSTATUS := tcpClient.ReceiveStatus;
    sLastReceiveSTATUS := tcpClient.LastReceiveStatus;
    sReceiveLength := tcpClient.ReceiveLength;
    sReceiveCount := tcpClient.ReceiveCount;
    sReceiveErrorCount := tcpClient.ReceiveErrorCount;

    // FOR i := DINT#0 TO DINT#63 DO
    //     sReceiveBuffer[i] := tcpClient.ReceiveBuffer[i];
    // END_FOR;


    tcp_praser.Execute(tcpClient.ReceiveBuffer, tcpClient.ReceiveLength);

    
// tcp_praser.ExtractedPayload[0] :=1;


    // DeserializeFromBytes(startOffset := UDINT#0, source := tcpClient.ReceiveBuffer, value := MI_staion_1);

    if tcp_praser.PayloadReady THEN

        DeserializeFromBytes(startOffset := UDINT#0, source := tcp_praser.ExtractedPayload, value := MI_staion_1);
    END_IF;


        if MI_staion_1.button_1_1_1 = TRUE THEN
    
        MO_staion_1.led_1_1_1 := TRUE;
        MO_staion_1.led_1_1_2 := FALSE;

    MO_staion_1.belt_1_1_1[0] := REAL#1.0;
    MO_staion_1.belt_1_1_2[0] := REAL#1.0;
    MO_staion_1.belt_1_1_3[0] := REAL#1.0;
    MO_staion_1.belt_1_1_4[0] := REAL#1.0;
    MO_staion_1.belt_1_1_5[0] := REAL#1.0;
    else
         MO_staion_1.led_1_1_1 := FALSE;
        MO_staion_1.led_1_1_2 := TRUE;
    MO_staion_1.belt_1_1_1[0] := REAL#0.0;
    MO_staion_1.belt_1_1_2[0] := REAL#0.0;
    MO_staion_1.belt_1_1_3[0] := REAL#0.0;
    MO_staion_1.belt_1_1_4[0] := REAL#0.0;
    MO_staion_1.belt_1_1_5[0] := REAL#0.0;
    END_IF;

    MO_pubLen := SerializeToBytes(
        startOffset := UDINT#0,
        value := MO_staion_1,
        dest := MO_pubBuffer
    );
    DeserializeFromBytes(startOffset := UDINT#0, source := MO_pubBuffer, value := MO_pubMirror);
    
    tcpClient.QueueSend(
                dataLength := MO_pubLen,
                data := MO_pubBuffer
    );




END_PROGRAM
