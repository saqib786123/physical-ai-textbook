import React from 'react';
import Chatbot from '@site/src/components/Chatbot';
import AuthUI from '@site/src/components/AuthUI';
import { AuthProvider } from '@site/src/context/AuthContext';

// Default implementation, that you can customize
export default function Root({ children }: { children: React.ReactNode }) {
    return (
        <AuthProvider>
            <AuthUI />
            {children}
            <Chatbot />
        </AuthProvider>
    );
}
