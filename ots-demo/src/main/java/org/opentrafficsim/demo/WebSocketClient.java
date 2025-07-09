package org.opentrafficsim.demo;

import org.json.JSONException;
import org.json.JSONObject;

import javax.websocket.*;
import java.net.URI;

@ClientEndpoint
public class WebSocketClient {
    private Session userSession;
    private MyListener listener;

    public WebSocketClient(URI endpointURI) {
        try {
            WebSocketContainer container = ContainerProvider.getWebSocketContainer();
            container.connectToServer(this, endpointURI);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void setListener(MyListener listener) {
        this.listener = listener;
    }

    @OnOpen
    public void onOpen(Session session) {
        System.out.println("Connected to server");
        this.userSession = session;
    }

    @OnMessage
    public void onMessage(String message) {
        try {
            JSONObject jsonObject = new JSONObject(message);
            if (listener != null) {
                listener.onEvent(jsonObject);
            }
        } catch (JSONException e) {
            // message is not a JSON
            System.out.println(message);
        }
    }

    @OnClose
    public void onClose(Session session, CloseReason reason) {
//        System.out.println("Disconnected: " + reason);
        this.userSession = null;
    }

    public void sendMessage(String message) {
        if (userSession != null && userSession.isOpen()) {
            userSession.getAsyncRemote().sendText(message);
        } else {
//            System.out.println("Session is not open.");
        }
    }
}

interface MyListener {
    void onEvent(JSONObject object);
}
