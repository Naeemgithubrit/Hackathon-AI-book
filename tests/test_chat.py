import requests
import json

# Test creating a session
response = requests.post('http://localhost:8000/api/session')
if response.status_code == 200:
    session_data = response.json()
    session_id = session_data['session_id']
    print(f'Created session: {session_id}')

    # Test chat with a simple question
    chat_data = {
        'session_id': session_id,
        'message': 'What is ROS2?'
    }
    headers = {'Content-Type': 'application/json'}
    chat_response = requests.post('http://localhost:8000/api/chat', json=chat_data, headers=headers, stream=True)

    if chat_response.status_code == 200:
        print('Chat response received')
        full_response = ""
        for line in chat_response.iter_lines():
            if line:
                line = line.decode('utf-8')
                if line.startswith('data: '):
                    data = line[6:]
                    if data != '[DONE]':
                        try:
                            event = json.loads(data)
                            if event.get('type') == 'token':
                                full_response += event['delta']
                            elif event.get('type') == 'error':
                                print(f'Error: {event.get("message", "Unknown error")}')
                        except json.JSONDecodeError:
                            pass
        print('Full response:')
        print(full_response)
    else:
        print(f'Chat failed: {chat_response.status_code} - {chat_response.text}')
else:
    print(f'Session creation failed: {response.status_code} - {response.text}')